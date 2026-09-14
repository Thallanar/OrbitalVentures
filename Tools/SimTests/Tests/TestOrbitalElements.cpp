#include "TestHarness.h"

#include "Orbital/Constants.h"
#include "Orbital/MathUtil.h"
#include "Orbital/OrbitalElements.h"

using namespace Orbital;

namespace
{
	constexpr double Mu = Constants::Earth::Mu;
	constexpr double Re = Constants::Earth::EquatorialRadius;

	double AngleError(double A, double B)
	{
		return std::abs(std::remainder(A - B, Constants::TwoPi));
	}

	FOrbitalElements MakeElements(double A, double E, double IncDeg, double RaanDeg, double ArgPDeg, double NuDeg)
	{
		return {A, E, DegToRad(IncDeg), DegToRad(RaanDeg), DegToRad(ArgPDeg), DegToRad(NuDeg)};
	}

	void CheckRoundTrip(const FOrbitalElements& In)
	{
		const FStateVector State = ElementsToState(In, Mu);
		const FOrbitalElements Out = StateToElements(State, Mu);
		OV_CHECK_NEAR(Out.SemiMajorAxis, In.SemiMajorAxis, 1e-9 * std::abs(In.SemiMajorAxis));
		OV_CHECK_NEAR(Out.Eccentricity, In.Eccentricity, 1e-10);
		OV_CHECK_NEAR(Out.Inclination, In.Inclination, 1e-10);
		OV_CHECK(AngleError(Out.RightAscensionOfAscendingNode, In.RightAscensionOfAscendingNode) < 1e-9);
		OV_CHECK(AngleError(Out.ArgumentOfPeriapsis, In.ArgumentOfPeriapsis) < 1e-9);
		OV_CHECK(AngleError(Out.TrueAnomaly, In.TrueAnomaly) < 1e-9);

		const FStateVector Again = ElementsToState(Out, Mu);
		OV_CHECK(Length(Again.Position - State.Position) < 1e-6);
		OV_CHECK(Length(Again.Velocity - State.Velocity) < 1e-9);
	}
}

OV_TEST(GeostationaryRadiusMatchesSiderealDay)
{
	const double SiderealDay = Constants::Earth::SiderealDay;
	const double N = Constants::TwoPi / SiderealDay;
	const double GeoRadius = std::cbrt(Mu / (N * N));
	OV_CHECK_NEAR(GeoRadius / 1000.0, 42164.17, 0.05);
	OV_CHECK_NEAR(OrbitalPeriod(42164.17e3, Mu), SiderealDay, 1.0);
}

OV_TEST(IssPeriodIsAbout93Minutes)
{
	// ISS real: ~92,7–93 min, dependendo da altitude do momento (~410–420 km).
	OV_CHECK_NEAR(OrbitalPeriod(Re + 420e3, Mu) / 60.0, 92.8, 0.4);
	OV_CHECK_NEAR(CircularOrbitSpeed(Re + 400e3, Mu), 7670.0, 10.0);
}

OV_TEST(ElementsRoundTripGeneralOrbits)
{
	CheckRoundTrip(MakeElements(7000e3, 0.01, 51.6, 30.0, 45.0, 60.0));   // LEO inclinada
	CheckRoundTrip(MakeElements(26600e3, 0.74, 63.4, 200.0, 270.0, 10.0)); // Molniya
	CheckRoundTrip(MakeElements(7078e3, 0.001, 98.2, 120.0, 90.0, 300.0)); // retrógrada (SSO)
	CheckRoundTrip(MakeElements(-20000e3, 1.5, 30.0, 10.0, 20.0, 30.0));   // hiperbólica
}

OV_TEST(ElementsRoundTripDegenerateOrbits)
{
	CheckRoundTrip(MakeElements(7000e3, 0.0, 45.0, 60.0, 0.0, 100.0));    // circular inclinada
	CheckRoundTrip(MakeElements(8000e3, 0.1, 0.0, 0.0, 75.0, 20.0));      // equatorial elíptica
	CheckRoundTrip(MakeElements(42164e3, 0.0, 0.0, 0.0, 0.0, 250.0));     // GEO
	CheckRoundTrip(MakeElements(9000e3, 0.2, 180.0, 0.0, 40.0, 10.0));    // equatorial retrógrada
}

OV_TEST(ApsisRadii)
{
	const FOrbitalElements Elements = MakeElements(7000e3, 0.01, 0.0, 0.0, 0.0, 0.0);
	OV_CHECK_NEAR(PeriapsisRadius(Elements), 6930e3, 1e-6);
	OV_CHECK_NEAR(ApoapsisRadius(Elements), 7070e3, 1e-6);
	OV_CHECK(std::isinf(ApoapsisRadius(MakeElements(-20000e3, 1.5, 0.0, 0.0, 0.0, 0.0))));
}
