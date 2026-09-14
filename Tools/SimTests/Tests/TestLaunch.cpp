#include <cmath>
#include <vector>

#include "TestHarness.h"

#include "Orbital/Constants.h"
#include "Orbital/MathUtil.h"
#include "ReferenceVehicles.h"
#include "Vehicle/LaunchSimulation.h"

using namespace Vehicle;

namespace
{
	constexpr double LaunchTestTargetAltitude = 200e3;
	// Tolerância de entrega: ±10 km, a ordem de grandeza das tolerâncias de injeção de contratos reais.
	constexpr double LaunchTestAltitudeTolerance = 10e3;

	double CapeLatitudeDegrees()
	{
		return Orbital::RadToDeg(ReferenceVehicles::MakeCapeCanaveral().Latitude);
	}

	FAscentProfile MakeProfile(double InclinationDegrees)
	{
		FAscentProfile Ascent;
		Ascent.TargetAltitude = LaunchTestTargetAltitude;
		Ascent.TargetInclination = Orbital::DegToRad(InclinationDegrees);
		return Ascent;
	}

	FLaunchSimulation RunLaunch(double PayloadMass, const FAscentProfile& Ascent, const std::vector<FEngineFailure>& Failures = {})
	{
		FLaunchSimulation Sim(ReferenceVehicles::MakeKestrel9(PayloadMass), ReferenceVehicles::MakeCapeCanaveral(), Ascent);
		for (const FEngineFailure& Failure : Failures)
		{
			Sim.ScheduleEngineFailure(Failure);
		}
		Sim.RunToCompletion();
		return Sim;
	}

	const FFlightEvent* FindEvent(const FLaunchSimulation& Sim, EFlightEventType Type, int Stage = -1)
	{
		for (const FFlightEvent& Event : Sim.GetEvents())
		{
			if (Event.Type == Type && (Stage < 0 || Event.StageIndex == Stage))
			{
				return &Event;
			}
		}
		return nullptr;
	}
}

OV_TEST(NominalLaunchReachesTargetOrbit)
{
	const FLaunchSimulation Sim = RunLaunch(10000.0, MakeProfile(CapeLatitudeDegrees()));
	const FTelemetry Final = Sim.GetTelemetry();

	OV_CHECK(Sim.GetOutcome() == EFlightOutcome::OrbitAchieved);
	OV_CHECK_NEAR(Final.PeriapsisAltitude, LaunchTestTargetAltitude, LaunchTestAltitudeTolerance);
	OV_CHECK_NEAR(Final.ApoapsisAltitude, LaunchTestTargetAltitude, LaunchTestAltitudeTolerance);
	OV_CHECK_NEAR(Orbital::RadToDeg(Final.Inclination), CapeLatitudeDegrees(), 0.05);
	OV_CHECK(Final.DeltaVRemaining > 0.0);
}

OV_TEST(NominalLaunchEventsHappenInOrder)
{
	const FLaunchSimulation Sim = RunLaunch(10000.0, MakeProfile(CapeLatitudeDegrees()));
	const FFlightEvent* Liftoff = FindEvent(Sim, EFlightEventType::Liftoff);
	const FFlightEvent* MaxQ = FindEvent(Sim, EFlightEventType::MaxQ);
	const FFlightEvent* BoosterCutoff = FindEvent(Sim, EFlightEventType::StageCutoff, 0);
	const FFlightEvent* Separation = FindEvent(Sim, EFlightEventType::StageSeparation, 0);
	const FFlightEvent* UpperIgnition = FindEvent(Sim, EFlightEventType::StageIgnition, 1);
	const FFlightEvent* Fairing = FindEvent(Sim, EFlightEventType::FairingJettison);
	const FFlightEvent* Insertion = FindEvent(Sim, EFlightEventType::OrbitInsertion);

	OV_CHECK(Liftoff && MaxQ && BoosterCutoff && Separation && UpperIgnition && Fairing && Insertion);
	if (!(Liftoff && MaxQ && BoosterCutoff && Separation && UpperIgnition && Fairing && Insertion))
	{
		return;
	}

	OV_CHECK(Liftoff->Time < MaxQ->Time);
	OV_CHECK(MaxQ->Time < BoosterCutoff->Time);
	OV_CHECK(BoosterCutoff->Time <= Separation->Time);
	OV_CHECK(Separation->Time < UpperIgnition->Time);
	OV_CHECK(UpperIgnition->Time < Insertion->Time);
	OV_CHECK(Fairing->Time < Insertion->Time);

	// Lançadores desta classe passam pelo max-Q com dezenas de kPa, cerca de um minuto após a decolagem.
	OV_CHECK(MaxQ->Value > 25e3 && MaxQ->Value < 50e3);
	OV_CHECK(MaxQ->Time > 40.0 && MaxQ->Time < 90.0);

	const std::vector<FFlightEvent>& Events = Sim.GetEvents();
	for (size_t Index = 1; Index < Events.size(); ++Index)
	{
		OV_CHECK(Events[Index - 1].Time <= Events[Index].Time);
	}
}

OV_TEST(IssInclinationLaunch)
{
	const FLaunchSimulation Sim = RunLaunch(10000.0, MakeProfile(51.6));
	OV_CHECK(Sim.GetOutcome() == EFlightOutcome::OrbitAchieved);
	OV_CHECK_NEAR(Orbital::RadToDeg(Sim.GetTelemetry().Inclination), 51.6, 0.05);
}

OV_TEST(EngineOutDuringBoostStillReachesOrbit)
{
	const FAscentProfile Ascent = MakeProfile(CapeLatitudeDegrees());
	const FLaunchSimulation Nominal = RunLaunch(10000.0, Ascent);
	const FLaunchSimulation Failed = RunLaunch(10000.0, Ascent, {{60.0, 0, 1}});

	OV_CHECK(Failed.GetOutcome() == EFlightOutcome::OrbitAchieved);
	OV_CHECK(FindEvent(Failed, EFlightEventType::EngineFailure, 0) != nullptr);

	// Com menos empuxo, o primeiro estágio leva mais tempo para queimar o mesmo propelente.
	const FFlightEvent* NominalCutoff = FindEvent(Nominal, EFlightEventType::StageCutoff, 0);
	const FFlightEvent* FailedCutoff = FindEvent(Failed, EFlightEventType::StageCutoff, 0);
	OV_CHECK(NominalCutoff && FailedCutoff && NominalCutoff->Time < FailedCutoff->Time);
}

OV_TEST(EngineOutAtLiftoffStillReachesOrbit)
{
	const FLaunchSimulation Sim = RunLaunch(10000.0, MakeProfile(CapeLatitudeDegrees()), {{1.0, 0, 1}});
	OV_CHECK(Sim.GetOutcome() == EFlightOutcome::OrbitAchieved);
}

OV_TEST(LosingTwoEnginesAtLiftoffLosesTheMission)
{
	const FLaunchSimulation Sim = RunLaunch(10000.0, MakeProfile(CapeLatitudeDegrees()), {{1.0, 0, 2}});
	OV_CHECK(Sim.GetOutcome() != EFlightOutcome::OrbitAchieved);
}

OV_TEST(AggressiveEarlyPitchoverBreaksTheStructure)
{
	FAscentProfile Ascent = MakeProfile(CapeLatitudeDegrees());
	Ascent.PitchoverAltitude = 300.0;
	Ascent.Aggressiveness = 1.0;
	const FLaunchSimulation Sim = RunLaunch(10000.0, Ascent);

	OV_CHECK(Sim.GetOutcome() == EFlightOutcome::StructuralFailure);
	const FFlightEvent* Failure = FindEvent(Sim, EFlightEventType::StructuralFailure);
	OV_CHECK(Failure && Failure->Value > ReferenceVehicles::MakeKestrel9(0.0).MaxDynamicPressure);
}

OV_TEST(TimidProfileWastesPropellantAgainstGravity)
{
	FAscentProfile Ascent = MakeProfile(CapeLatitudeDegrees());
	Ascent.Aggressiveness = 0.0;
	const FLaunchSimulation Sim = RunLaunch(10000.0, Ascent);
	OV_CHECK(Sim.GetOutcome() == EFlightOutcome::PropellantExhausted);
}

OV_TEST(PayloadMassLimitsTheVehicle)
{
	const FAscentProfile Ascent = MakeProfile(CapeLatitudeDegrees());
	OV_CHECK(RunLaunch(20000.0, Ascent).GetOutcome() == EFlightOutcome::OrbitAchieved);
	OV_CHECK(RunLaunch(30000.0, Ascent).GetOutcome() == EFlightOutcome::PropellantExhausted);
}

OV_TEST(DeltaVRemainingMatchesRocketEquationAtLiftoff)
{
	const FVehicleDef Definition = ReferenceVehicles::MakeKestrel9(10000.0);
	const FLaunchSimulation Sim(Definition, ReferenceVehicles::MakeCapeCanaveral(), MakeProfile(CapeLatitudeDegrees()));

	const FStageDef& Booster = Definition.Stages[0];
	const FStageDef& Upper = Definition.Stages[1];
	const double G0 = Orbital::Constants::StandardGravity;
	const double UpperMass = Upper.DryMass + Upper.PropellantMass + Definition.PayloadMass + Definition.FairingMass;
	const double LiftoffMass = UpperMass + Booster.DryMass + Booster.PropellantMass;
	const double Expected = Booster.Engine.VacuumIsp * G0 * std::log(LiftoffMass / (LiftoffMass - Booster.PropellantMass))
		+ Upper.Engine.VacuumIsp * G0 * std::log(UpperMass / (UpperMass - Upper.PropellantMass));

	OV_CHECK_NEAR(Sim.GetTelemetry().DeltaVRemaining, Expected, 1e-6);
}

OV_TEST(TimeWarpDoesNotChangeTheFlight)
{
	const FAscentProfile Ascent = MakeProfile(CapeLatitudeDegrees());
	FLaunchSimulation RealTime(ReferenceVehicles::MakeKestrel9(10000.0), ReferenceVehicles::MakeCapeCanaveral(), Ascent);
	FLaunchSimulation Warped(ReferenceVehicles::MakeKestrel9(10000.0), ReferenceVehicles::MakeCapeCanaveral(), Ascent);

	// 300 s em blocos de 5 s contra quadros de 60 Hz: atravessa a separação de estágios.
	for (int Chunk = 0; Chunk < 60; ++Chunk)
	{
		Warped.Advance(5.0);
	}
	while (RealTime.GetStepCount() < Warped.GetStepCount())
	{
		RealTime.Advance(1.0 / 60.0);
	}

	OV_CHECK(RealTime.GetStepCount() == Warped.GetStepCount());
	OV_CHECK(RealTime.GetStepCount() == 6000);
	const Orbital::FStateVector& A = RealTime.GetState();
	const Orbital::FStateVector& B = Warped.GetState();
	OV_CHECK(A.Position.X == B.Position.X && A.Position.Y == B.Position.Y && A.Position.Z == B.Position.Z);
	OV_CHECK(A.Velocity.X == B.Velocity.X && A.Velocity.Y == B.Velocity.Y && A.Velocity.Z == B.Velocity.Z);
	OV_CHECK(RealTime.GetEvents().size() == Warped.GetEvents().size());
}
