#include <algorithm>

#include "TestHarness.h"

#include "Orbital/Constants.h"
#include "Orbital/Gravity.h"
#include "Orbital/Integrator.h"
#include "Orbital/Kepler.h"
#include "Orbital/MathUtil.h"
#include "Orbital/OrbitalElements.h"

using namespace Orbital;

namespace
{
	constexpr double Mu = Constants::Earth::Mu;
	constexpr double Re = Constants::Earth::EquatorialRadius;

	FStateVector MakeState(double A, double E, double IncDeg, double RaanDeg, double ArgPDeg, double NuDeg)
	{
		return ElementsToState({A, E, DegToRad(IncDeg), DegToRad(RaanDeg), DegToRad(ArgPDeg), DegToRad(NuDeg)}, Mu);
	}
}

OV_TEST(KeplerSolverConverges)
{
	for (const double Ecc : {0.0, 0.3, 0.7, 0.9, 0.99})
	{
		for (const double M : {-3.0, -1.0, 0.0, 0.1, 1.0, 3.1, 10.0, 100.0})
		{
			const double E = SolveKeplerEquation(M, Ecc);
			OV_CHECK_NEAR(E - Ecc * std::sin(E), M, 1e-12);
		}
	}
}

OV_TEST(KeplerFullPeriodReturnsToStart)
{
	const FStateVector Start = MakeState(26600e3, 0.74, 63.4, 200.0, 270.0, 10.0);
	const double Period = OrbitalPeriod(26600e3, Mu);
	for (const double Revolutions : {1.0, 10.0, 1000.0})
	{
		const FStateVector End = PropagateKepler(Start, Mu, Revolutions * Period);
		OV_CHECK(Length(End.Position - Start.Position) < 1e-2);
	}
}

OV_TEST(KeplerMatchesMeanAnomalyPropagation)
{
	const double A = 26600e3;
	const double Ecc = 0.74;
	const FOrbitalElements Initial{A, Ecc, DegToRad(63.4), DegToRad(200.0), DegToRad(270.0), DegToRad(10.0)};
	const FStateVector Start = ElementsToState(Initial, Mu);

	for (const double Dt : {60.0, 12345.0, -5000.0, 1e6})
	{
		const double MeanMotion = std::sqrt(Mu / (A * A * A));
		const double M = TrueToMeanAnomaly(Initial.TrueAnomaly, Ecc) + MeanMotion * Dt;
		FOrbitalElements Expected = Initial;
		Expected.TrueAnomaly = EccentricToTrueAnomaly(SolveKeplerEquation(M, Ecc), Ecc);

		const FStateVector Propagated = PropagateKepler(Start, Mu, Dt);
		OV_CHECK(Length(Propagated.Position - ElementsToState(Expected, Mu).Position) < 1e-2);
	}
}

OV_TEST(KeplerHyperbolicIsReversible)
{
	const FStateVector Start = MakeState(-20000e3, 1.5, 30.0, 10.0, 20.0, -60.0);
	const FStateVector Forward = PropagateKepler(Start, Mu, 86400.0);
	const FStateVector Back = PropagateKepler(Forward, Mu, -86400.0);

	OV_CHECK(Length(Forward.Position) > 10.0 * Length(Start.Position));
	OV_CHECK_NEAR(SpecificOrbitalEnergy(Forward, Mu), SpecificOrbitalEnergy(Start, Mu),
		1e-10 * std::abs(SpecificOrbitalEnergy(Start, Mu)));
	OV_CHECK(Length(Back.Position - Start.Position) < 1e-2);
}

OV_TEST(RK4MatchesKeplerOverOneOrbit)
{
	const FStateVector Start = MakeState(Re + 400e3, 0.001, 51.6, 0.0, 0.0, 0.0);
	const double Dt = 1.0;
	const int Steps = static_cast<int>(OrbitalPeriod(Re + 400e3, Mu) / Dt);

	FStateVector State = Start;
	for (int Step = 0; Step < Steps; ++Step)
	{
		State = StepRK4(State, Step * Dt, Dt, [](double, const FStateVector& S) { return PointMassAcceleration(S.Position, Mu); });
	}

	const FStateVector Reference = PropagateKepler(Start, Mu, Steps * Dt);
	OV_CHECK(Length(State.Position - Reference.Position) < 1e-2);
}

OV_TEST(VerletEnergyBoundedOver1000Orbits)
{
	const double A = Re + 400e3;
	const FStateVector Start = MakeState(A, 0.01, 51.6, 0.0, 0.0, 0.0);
	const double InitialEnergy = SpecificOrbitalEnergy(Start, Mu);
	const double Dt = 2.0;
	const int StepsPerOrbit = static_cast<int>(OrbitalPeriod(A, Mu) / Dt);
	const int Orbits = 1000;
	const auto Accel = [](const FVec3& Position) { return PointMassAcceleration(Position, Mu); };

	double MaxErrorFirstOrbits = 0.0;
	double MaxErrorLastOrbits = 0.0;
	FStateVector State = Start;
	for (int Step = 0; Step < Orbits * StepsPerOrbit; ++Step)
	{
		State = StepVelocityVerlet(State, Dt, Accel);
		const double RelativeError = std::abs(SpecificOrbitalEnergy(State, Mu) / InitialEnergy - 1.0);
		const int Orbit = Step / StepsPerOrbit;
		if (Orbit < 10)
		{
			MaxErrorFirstOrbits = std::max(MaxErrorFirstOrbits, RelativeError);
		}
		else if (Orbit >= Orbits - 10)
		{
			MaxErrorLastOrbits = std::max(MaxErrorLastOrbits, RelativeError);
		}
	}

	// Simplético: o erro oscila mas não cresce entre o início e o fim.
	OV_CHECK(MaxErrorFirstOrbits < 1e-6);
	OV_CHECK(MaxErrorLastOrbits < 1.1 * MaxErrorFirstOrbits);
}
