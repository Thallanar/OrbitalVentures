#include "TestHarness.h"

#include "Orbital/Constants.h"
#include "Orbital/Gravity.h"
#include "Orbital/Integrator.h"
#include "Orbital/Maneuvers.h"
#include "Orbital/MathUtil.h"
#include "Orbital/OrbitalElements.h"

using namespace Orbital;
using namespace Orbital::Constants;

OV_TEST(SpheresOfInfluence)
{
	OV_CHECK_NEAR(SphereOfInfluenceRadius(Moon::SemiMajorAxis, Moon::Mu, Earth::Mu) / 1000.0, 66100.0, 500.0);
	OV_CHECK_NEAR(SphereOfInfluenceRadius(AstronomicalUnit, Earth::Mu, Sun::Mu) / 1000.0, 925000.0, 5000.0);
}

OV_TEST(SunSynchronousInclinationAt700Km)
{
	OV_CHECK_NEAR(RadToDeg(SunSynchronousInclination(Earth::EquatorialRadius + 700e3, 0.0)), 98.19, 0.03);
	OV_CHECK(std::isnan(SunSynchronousInclination(Earth::EquatorialRadius + 10000e3, 0.0)));
}

OV_TEST(J2PullsHarderAtEquator)
{
	const FVec3 AtEquator = J2Acceleration({Earth::EquatorialRadius + 400e3, 0.0, 0.0}, Earth::Mu, Earth::J2, Earth::EquatorialRadius);
	const FVec3 AtPole = J2Acceleration({0.0, 0.0, Earth::EquatorialRadius + 400e3}, Earth::Mu, Earth::J2, Earth::EquatorialRadius);
	OV_CHECK(AtEquator.X < 0.0);
	OV_CHECK(AtPole.Z > 0.0);
}

OV_TEST(J2NodalPrecessionMatchesNumericalIntegration)
{
	const double A = Earth::EquatorialRadius + 700e3;
	const double Inclination = SunSynchronousInclination(A, 0.0);
	const FStateVector Start = ElementsToState({A, 0.0, Inclination, 0.0, 0.0, 0.0}, Earth::Mu);
	const auto Accel = [](double, const FStateVector& S)
	{
		return PointMassAcceleration(S.Position, Earth::Mu)
			+ J2Acceleration(S.Position, Earth::Mu, Earth::J2, Earth::EquatorialRadius);
	};

	const double Dt = 5.0;
	const double Duration = 86400.0;
	FStateVector State = Start;
	for (double Time = 0.0; Time < Duration; Time += Dt)
	{
		State = StepRK4(State, Time, Dt, Accel);
	}

	const double NumericalDrift = StateToElements(State, Earth::Mu).RightAscensionOfAscendingNode;
	const double AnalyticalDrift = NodalPrecessionRate(A, 0.0, Inclination, Earth::Mu, Earth::J2, Earth::EquatorialRadius) * Duration;
	OV_CHECK_NEAR(RadToDeg(AnalyticalDrift), 360.0 / 365.24219, 1e-6);
	OV_CHECK_NEAR(RadToDeg(NumericalDrift), RadToDeg(AnalyticalDrift), 0.03);
}

OV_TEST(HohmannLeoToGeo)
{
	const FHohmannTransfer Transfer = ComputeHohmannTransfer(Earth::EquatorialRadius + 300e3, 42164.17e3, Earth::Mu);
	OV_CHECK_NEAR(Transfer.TotalDeltaV, 3893.0, 10.0);
	OV_CHECK_NEAR(Transfer.TransferTime / 3600.0, 5.27, 0.05);
	OV_CHECK_NEAR(Transfer.DeltaV1 + Transfer.DeltaV2, Transfer.TotalDeltaV, 1e-9);
}

OV_TEST(PlaneChange)
{
	OV_CHECK_NEAR(PlaneChangeDeltaV(7500.0, DegToRad(60.0)), 7500.0, 1e-9);
	OV_CHECK_NEAR(PlaneChangeDeltaV(7500.0, 0.0), 0.0, 1e-12);
}
