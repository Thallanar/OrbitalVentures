#include "Vehicle/LaunchSimulation.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "Orbital/Atmosphere.h"
#include "Orbital/Constants.h"
#include "Orbital/Geodesy.h"
#include "Orbital/Gravity.h"
#include "Orbital/Integrator.h"
#include "Orbital/OrbitalElements.h"
#include "Vehicle/Aerodynamics.h"

namespace Vehicle
{
	namespace
	{
		constexpr double LaunchStagingCoastDuration = 3.0;         // s entre a separação e a ignição seguinte
		constexpr double LaunchGuidanceFreezeTime = 5.0;          // s: perto do corte, a direção fica congelada
		constexpr double LaunchMinStablePeriapsisAltitude = 120e3; // m
		constexpr double LaunchDragCutoffAltitude = 1000e3;        // m
		constexpr double LaunchMaxQMinimum = 1000.0;               // Pa, abaixo disso não há max-Q a relatar
		constexpr double LaunchMaxQReportFraction = 0.95;          // relata o pico quando Q cai 5% abaixo dele
		// Corredor de subida: ângulo de trajetória mínimo na fase atmosférica, como as restrições de atitude de um
		// lançador real. Constante até CorridorFlatAltitude e relaxando linearmente até zerar em CorridorEndAltitude.
		constexpr double LaunchCorridorMaxAngle = 0.25 * Orbital::Constants::Pi; // 45°
		constexpr double LaunchCorridorFlatAltitude = 10e3;                      // m
		constexpr double LaunchCorridorEndAltitude = 60e3;                       // m

		constexpr double EarthMu = Orbital::Constants::Earth::Mu;
		constexpr double EarthRadius = Orbital::Constants::Earth::EquatorialRadius;
		constexpr Orbital::FVec3 EarthAngularVelocity(0.0, 0.0, Orbital::Constants::Earth::RotationRate);

		bool IsPoweredPhase(EFlightPhase FlightPhase)
		{
			return FlightPhase == EFlightPhase::VerticalAscent || FlightPhase == EFlightPhase::Pitchover
				|| FlightPhase == EFlightPhase::GravityTurn || FlightPhase == EFlightPhase::ClosedLoop;
		}

		Orbital::FVec3 AirRelativeVelocity(const Orbital::FStateVector& S)
		{
			return S.Velocity - Orbital::Cross(EarthAngularVelocity, S.Position);
		}

		double AscentCorridorMinFlightPath(double Altitude)
		{
			if (Altitude <= LaunchCorridorFlatAltitude)
			{
				return LaunchCorridorMaxAngle;
			}
			const double Fraction = (LaunchCorridorEndAltitude - Altitude) / (LaunchCorridorEndAltitude - LaunchCorridorFlatAltitude);
			return LaunchCorridorMaxAngle * std::max(0.0, Fraction);
		}

		Orbital::FVec3 AerodynamicDrag(const Orbital::FStateVector& S, double Area)
		{
			const double Altitude = Orbital::GeodeticHeight(S.Position);
			if (Altitude >= LaunchDragCutoffAltitude)
			{
				return {};
			}
			const Orbital::FAtmosphereSample Air = Orbital::SampleEarthAtmosphere(std::max(0.0, Altitude));
			const Orbital::FVec3 AirVelocity = AirRelativeVelocity(S);
			const double Airspeed = Orbital::Length(AirVelocity);
			if (Airspeed <= 0.0 || Air.Density <= 0.0)
			{
				return {};
			}
			const double Coefficient = DragCoefficient(Airspeed / Air.SpeedOfSound);
			return AirVelocity * (-0.5 * Air.Density * Airspeed * Coefficient * Area);
		}
	}

	FLaunchSimulation::FLaunchSimulation(FVehicleDef InDefinition, FLaunchSite InSite, FAscentProfile InProfile)
		: Definition(std::move(InDefinition))
		, Site(std::move(InSite))
		, Profile(InProfile)
	{
		// No instante zero, o referencial inercial coincide com o fixo na Terra.
		State.Position = Orbital::GeodeticToCartesian({Site.Latitude, Site.Longitude, Site.Height});
		State.Velocity = Orbital::Cross(EarthAngularVelocity, State.Position);

		Plane = ComputeLaunchPlane(State.Position, Profile.TargetInclination);
		KickAngle = PitchoverKickAngle(Profile.Aggressiveness);
		TargetRadius = EarthRadius + Profile.TargetAltitude;

		// O plano alvo é inercial, mas o foguete sai de uma Terra que gira. O rumo de subida relativo ao solo
		// desconta a velocidade do sítio para que, em velocidade orbital, a velocidade inercial caia no plano.
		{
			const Orbital::FVec3 Up = Orbital::GeodeticUp(State.Position);
			const Orbital::FVec3 East = Orbital::Normalized(Orbital::FVec3(-Up.Y, Up.X, 0.0));
			const Orbital::FVec3 North = Orbital::Cross(Up, East);
			// A direção de lançamento pertence ao plano orbital, que passa pelo centro da Terra: usa a radial.
			const Orbital::FVec3 InertialDirection = Orbital::Normalized(Orbital::Cross(Plane.Normal, Orbital::Normalized(State.Position)));
			const double OrbitalSpeed = std::sqrt(EarthMu / TargetRadius);
			const double EastSpeed = OrbitalSpeed * Orbital::Dot(InertialDirection, East) - Orbital::Length(State.Velocity);
			const double NorthSpeed = OrbitalSpeed * Orbital::Dot(InertialDirection, North);
			LaunchHeading = std::atan2(EastSpeed, NorthSpeed);
		}

		for (const FStageDef& Stage : Definition.Stages)
		{
			StagePropellant.push_back(Stage.PropellantMass);
			StageEngines.push_back(Stage.EngineCount);
		}
		bFairingAttached = Definition.FairingMass > 0.0;
		LastDirection = Orbital::GeodeticUp(State.Position);
	}

	void FLaunchSimulation::ScheduleEngineFailure(const FEngineFailure& Failure)
	{
		PendingFailures.push_back(Failure);
	}

	int FLaunchSimulation::Advance(double DeltaSeconds)
	{
		Accumulator += DeltaSeconds;
		// A pequena folga evita perder um passo quando o acumulador fica a um erro de arredondamento do múltiplo.
		const int Steps = static_cast<int>(std::floor(Accumulator / FixedTimeStep + 1e-9));
		Accumulator = std::max(0.0, Accumulator - Steps * FixedTimeStep);
		for (int Index = 0; Index < Steps; ++Index)
		{
			Step();
		}
		return Steps;
	}

	void FLaunchSimulation::RunToCompletion(double MaxDuration)
	{
		while (Outcome == EFlightOutcome::InFlight && Time < MaxDuration)
		{
			Step();
		}
	}

	void FLaunchSimulation::Step()
	{
		if (Outcome != EFlightOutcome::InFlight)
		{
			return;
		}

		ApplyDueEngineFailures();

		const double Dt = FixedTimeStep;
		const double StartMass = CurrentMass();
		const double StartAltitude = Orbital::GeodeticHeight(State.Position);
		const double AmbientPressure = Orbital::SampleEarthAtmosphere(std::max(0.0, StartAltitude)).Pressure;
		const bool bBurning = IsPoweredPhase(Phase) && StagePropellant[StageIndex] > 0.0 && StageEngines[StageIndex] > 0;

		double Throttle = 0.0;
		double Thrust = 0.0;
		double MassFlow = 0.0;
		Orbital::FVec3 Direction = LastDirection;
		if (bBurning)
		{
			const FStageDef& Stage = Definition.Stages[StageIndex];
			const double FullThrust = StageEngines[StageIndex] * ThrustAtPressure(Stage.Engine, AmbientPressure);
			Throttle = 1.0;
			if (Profile.MaxAcceleration > 0.0 && FullThrust > 0.0)
			{
				Throttle = std::clamp(Profile.MaxAcceleration * StartMass / FullThrust, Stage.Engine.MinThrottle, 1.0);
			}
			Thrust = Throttle * FullThrust;
			MassFlow = Throttle * StageEngines[StageIndex] * MassFlowRate(Stage.Engine);
			Direction = ComputeThrustDirection(Thrust / StartMass);
		}

		if (!bLiftedOff)
		{
			const double Weight = StartMass * EarthMu / Orbital::LengthSquared(State.Position);
			if (Thrust <= Weight)
			{
				// Ainda preso à plataforma: gira com a Terra e queima propelente sem sair do chão.
				const double Angle = Orbital::Constants::Earth::RotationRate * Dt;
				const Orbital::FVec3 P = State.Position;
				State.Position = Orbital::FVec3(
					P.X * std::cos(Angle) - P.Y * std::sin(Angle),
					P.X * std::sin(Angle) + P.Y * std::cos(Angle),
					P.Z);
				State.Velocity = Orbital::Cross(EarthAngularVelocity, State.Position);
				if (bBurning)
				{
					StagePropellant[StageIndex] = std::max(0.0, StagePropellant[StageIndex] - MassFlow * Dt);
				}
				Time += Dt;
				++StepCount;
				LastThrottle = Throttle;
				LastThrustAcceleration = Thrust / StartMass;
				if (!bBurning || StagePropellant[StageIndex] <= 0.0)
				{
					Outcome = EFlightOutcome::PropellantExhausted;
					Phase = EFlightPhase::Finished;
				}
				return;
			}
			bLiftedOff = true;
			AddEvent(EFlightEventType::Liftoff, StageIndex);
		}

		const double StartTime = Time;
		const double DragArea = ReferenceArea();
		const Orbital::FVec3 ThrustDirection = Direction;
		auto Acceleration = [&](double Tau, const Orbital::FStateVector& S)
		{
			const double MassNow = StartMass - MassFlow * (Tau - StartTime);
			Orbital::FVec3 Accel = Orbital::PointMassAcceleration(S.Position, EarthMu)
				+ Orbital::J2Acceleration(S.Position, EarthMu, Orbital::Constants::Earth::J2, EarthRadius);
			if (Thrust > 0.0)
			{
				Accel += ThrustDirection * (Thrust / MassNow);
			}
			Accel += AerodynamicDrag(S, DragArea) / MassNow;
			return Accel;
		};
		State = Orbital::StepRK4(State, StartTime, Dt, Acceleration);
		Time += Dt;
		++StepCount;

		if (bBurning)
		{
			StagePropellant[StageIndex] = std::max(0.0, StagePropellant[StageIndex] - MassFlow * Dt);
			LastDirection = Direction;
		}
		LastThrottle = Throttle;
		LastThrustAcceleration = Thrust / StartMass;

		UpdateFlightState();
	}

	Orbital::FVec3 FLaunchSimulation::ComputeThrustDirection(double ThrustAcceleration)
	{
		switch (Phase)
		{
		case EFlightPhase::VerticalAscent:
			return Orbital::GeodeticUp(State.Position);
		case EFlightPhase::Pitchover:
			return Orbital::GeodeticUp(State.Position) * std::cos(KickAngle) + LaunchHeadingDirection() * std::sin(KickAngle);
		case EFlightPhase::GravityTurn:
		{
			// Giro gravitacional: empuxo alinhado à velocidade relativa ao ar, com ângulo de ataque nulo. O corredor
			// impede que um veículo com pouco empuxo, como um que perdeu motor na decolagem, deite no ar denso.
			const Orbital::FVec3 Up = Orbital::GeodeticUp(State.Position);
			const Orbital::FVec3 AirVelocity = AirRelativeVelocity(State);
			const double MinFlightPath = AscentCorridorMinFlightPath(Orbital::GeodeticHeight(State.Position));
			if (AirFlightPathAngle() >= MinFlightPath)
			{
				return Orbital::Normalized(AirVelocity);
			}
			const Orbital::FVec3 Horizontal = AirVelocity - Up * Orbital::Dot(AirVelocity, Up);
			const Orbital::FVec3 Heading = Orbital::LengthSquared(Horizontal) > 1e-6
				? Orbital::Normalized(Horizontal)
				: LaunchHeadingDirection();
			return Up * std::sin(MinFlightPath) + Heading * std::cos(MinFlightPath);
		}
		case EFlightPhase::ClosedLoop:
		{
			FClosedLoopInput Input;
			Input.State = State;
			Input.PlaneNormal = Plane.Normal;
			Input.TargetRadius = TargetRadius;
			Input.Mu = EarthMu;
			Input.ThrustAcceleration = ThrustAcceleration;
			Input.ExhaustVelocity = Definition.Stages[StageIndex].Engine.VacuumIsp * Orbital::Constants::StandardGravity;
			const FClosedLoopCommand Command = ComputeClosedLoopCommand(Input);
			if (bClosedLoopEngaged && Command.TimeToGo < LaunchGuidanceFreezeTime)
			{
				return LastDirection;
			}
			bClosedLoopEngaged = true;
			return Command.Direction;
		}
		default:
			return LastDirection;
		}
	}

	void FLaunchSimulation::UpdateFlightState()
	{
		const double Altitude = Orbital::GeodeticHeight(State.Position);
		if (Altitude < 0.0)
		{
			AddEvent(EFlightEventType::Impact, StageIndex);
			Outcome = EFlightOutcome::Impact;
			Phase = EFlightPhase::Finished;
			return;
		}

		const double Airspeed = Orbital::Length(AirRelativeVelocity(State));
		const double DynamicPressure = 0.5 * Orbital::SampleEarthAtmosphere(Altitude).Density * Airspeed * Airspeed;
		if (Definition.MaxDynamicPressure > 0.0 && DynamicPressure > Definition.MaxDynamicPressure)
		{
			AddEvent(EFlightEventType::StructuralFailure, StageIndex, DynamicPressure);
			Outcome = EFlightOutcome::StructuralFailure;
			Phase = EFlightPhase::Finished;
			return;
		}

		if (!bMaxQReported)
		{
			if (DynamicPressure > PeakDynamicPressure)
			{
				PeakDynamicPressure = DynamicPressure;
				PeakDynamicPressureTime = Time;
			}
			else if (PeakDynamicPressure > LaunchMaxQMinimum && DynamicPressure < LaunchMaxQReportFraction * PeakDynamicPressure)
			{
				// O pico só é reconhecido depois que passa: o evento entra com o horário do pico, na ordem certa.
				const FFlightEvent MaxQ{PeakDynamicPressureTime, EFlightEventType::MaxQ, StageIndex, PeakDynamicPressure};
				const auto Position = std::upper_bound(Events.begin(), Events.end(), MaxQ,
					[](const FFlightEvent& A, const FFlightEvent& B) { return A.Time < B.Time; });
				Events.insert(Position, MaxQ);
				bMaxQReported = true;
			}
		}

		if (bFairingAttached && Altitude >= Definition.FairingJettisonAltitude)
		{
			bFairingAttached = false;
			AddEvent(EFlightEventType::FairingJettison, StageIndex);
		}

		switch (Phase)
		{
		case EFlightPhase::VerticalAscent:
			if (Altitude >= Profile.PitchoverAltitude)
			{
				Phase = EFlightPhase::Pitchover;
				AddEvent(EFlightEventType::PitchoverStart, StageIndex);
			}
			break;
		case EFlightPhase::Pitchover:
			if (AirFlightPathAngle() <= 0.5 * Orbital::Constants::Pi - KickAngle)
			{
				Phase = EFlightPhase::GravityTurn;
			}
			break;
		case EFlightPhase::StagingCoast:
			StagingTimer -= FixedTimeStep;
			if (StagingTimer <= 0.0)
			{
				Phase = EFlightPhase::ClosedLoop;
				AddEvent(EFlightEventType::StageIgnition, StageIndex);
			}
			break;
		case EFlightPhase::ClosedLoop:
			if (Orbital::SpecificOrbitalEnergy(State, EarthMu) >= -EarthMu / (2.0 * TargetRadius))
			{
				CutOffAtTargetEnergy();
				return;
			}
			break;
		default:
			break;
		}

		if (IsPoweredPhase(Phase) && (StagePropellant[StageIndex] <= 0.0 || StageEngines[StageIndex] <= 0))
		{
			HandleStageBurnout();
		}
	}

	void FLaunchSimulation::HandleStageBurnout()
	{
		AddEvent(EFlightEventType::StageCutoff, StageIndex);
		LastThrottle = 0.0;
		LastThrustAcceleration = 0.0;

		if (StageIndex + 1 >= static_cast<int>(Definition.Stages.size()))
		{
			Outcome = EFlightOutcome::PropellantExhausted;
			Phase = EFlightPhase::Finished;
			return;
		}

		AddEvent(EFlightEventType::StageSeparation, StageIndex);
		++StageIndex;
		Phase = EFlightPhase::StagingCoast;
		StagingTimer = LaunchStagingCoastDuration;
	}

	void FLaunchSimulation::CutOffAtTargetEnergy()
	{
		AddEvent(EFlightEventType::OrbitInsertion, StageIndex);
		LastThrottle = 0.0;
		LastThrustAcceleration = 0.0;
		const Orbital::FOrbitalElements Elements = Orbital::StateToElements(State, EarthMu);
		const double PeriapsisAltitude = Orbital::PeriapsisRadius(Elements) - EarthRadius;
		Outcome = PeriapsisAltitude >= LaunchMinStablePeriapsisAltitude
			? EFlightOutcome::OrbitAchieved
			: EFlightOutcome::SuborbitalCutoff;
		Phase = EFlightPhase::Finished;
	}

	void FLaunchSimulation::ApplyDueEngineFailures()
	{
		auto IsDue = [this](const FEngineFailure& Failure) { return Failure.Time <= Time; };
		for (const FEngineFailure& Failure : PendingFailures)
		{
			if (IsDue(Failure) && Failure.StageIndex >= 0 && Failure.StageIndex < static_cast<int>(StageEngines.size()))
			{
				StageEngines[Failure.StageIndex] = std::max(0, StageEngines[Failure.StageIndex] - Failure.EnginesLost);
				AddEvent(EFlightEventType::EngineFailure, Failure.StageIndex, Failure.EnginesLost);
			}
		}
		PendingFailures.erase(std::remove_if(PendingFailures.begin(), PendingFailures.end(), IsDue), PendingFailures.end());
	}

	void FLaunchSimulation::AddEvent(EFlightEventType Type, int EventStage, double Value)
	{
		Events.push_back({Time, Type, EventStage, Value});
	}

	double FLaunchSimulation::CurrentMass() const
	{
		double Mass = Definition.PayloadMass + (bFairingAttached ? Definition.FairingMass : 0.0);
		for (size_t Index = StageIndex; Index < Definition.Stages.size(); ++Index)
		{
			Mass += Definition.Stages[Index].DryMass + StagePropellant[Index];
		}
		return Mass;
	}

	double FLaunchSimulation::ReferenceArea() const
	{
		double Diameter = 0.0;
		for (size_t Index = StageIndex; Index < Definition.Stages.size(); ++Index)
		{
			Diameter = std::max(Diameter, Definition.Stages[Index].Diameter);
		}
		return 0.25 * Orbital::Constants::Pi * Diameter * Diameter;
	}

	double FLaunchSimulation::DeltaVRemaining() const
	{
		double Total = 0.0;
		for (size_t Index = StageIndex; Index < Definition.Stages.size(); ++Index)
		{
			if (StageEngines[Index] <= 0 || StagePropellant[Index] <= 0.0)
			{
				continue;
			}
			double IgnitionMass = Definition.PayloadMass + (bFairingAttached ? Definition.FairingMass : 0.0);
			for (size_t Above = Index; Above < Definition.Stages.size(); ++Above)
			{
				IgnitionMass += Definition.Stages[Above].DryMass + StagePropellant[Above];
			}
			const double ExhaustVelocity = Definition.Stages[Index].Engine.VacuumIsp * Orbital::Constants::StandardGravity;
			Total += ExhaustVelocity * std::log(IgnitionMass / (IgnitionMass - StagePropellant[Index]));
		}
		return Total;
	}

	double FLaunchSimulation::AirFlightPathAngle() const
	{
		const Orbital::FVec3 AirVelocity = AirRelativeVelocity(State);
		const double Airspeed = Orbital::Length(AirVelocity);
		if (Airspeed <= 0.0)
		{
			return 0.5 * Orbital::Constants::Pi;
		}
		return std::asin(std::clamp(Orbital::Dot(AirVelocity, Orbital::GeodeticUp(State.Position)) / Airspeed, -1.0, 1.0));
	}

	Orbital::FVec3 FLaunchSimulation::LaunchHeadingDirection() const
	{
		const Orbital::FVec3 Up = Orbital::GeodeticUp(State.Position);
		const Orbital::FVec3 East = Orbital::Normalized(Orbital::FVec3(-Up.Y, Up.X, 0.0));
		const Orbital::FVec3 North = Orbital::Cross(Up, East);
		return East * std::sin(LaunchHeading) + North * std::cos(LaunchHeading);
	}

	FTelemetry FLaunchSimulation::GetTelemetry() const
	{
		FTelemetry Out;
		Out.Time = Time;
		Out.Phase = Phase;
		Out.StageIndex = StageIndex;
		Out.ActiveEngines = StageEngines[StageIndex];
		Out.Altitude = Orbital::GeodeticHeight(State.Position);
		Out.InertialSpeed = Orbital::Length(State.Velocity);
		Out.Airspeed = Orbital::Length(AirRelativeVelocity(State));

		const Orbital::FAtmosphereSample Air = Orbital::SampleEarthAtmosphere(std::max(0.0, Out.Altitude));
		Out.Mach = Out.Airspeed / Air.SpeedOfSound;
		Out.DynamicPressure = 0.5 * Air.Density * Out.Airspeed * Out.Airspeed;
		Out.ThrustAcceleration = LastThrustAcceleration;
		Out.Throttle = LastThrottle;
		Out.PitchAngle = std::asin(std::clamp(Orbital::Dot(LastDirection, Orbital::GeodeticUp(State.Position)), -1.0, 1.0));
		Out.Mass = CurrentMass();
		Out.DeltaVRemaining = DeltaVRemaining();

		const Orbital::FOrbitalElements Elements = Orbital::StateToElements(State, EarthMu);
		Out.PeriapsisAltitude = Orbital::PeriapsisRadius(Elements) - EarthRadius;
		const double Apoapsis = Orbital::ApoapsisRadius(Elements);
		Out.ApoapsisAltitude = std::isinf(Apoapsis) ? std::numeric_limits<double>::infinity() : Apoapsis - EarthRadius;
		Out.Inclination = Elements.Inclination;
		return Out;
	}
}
