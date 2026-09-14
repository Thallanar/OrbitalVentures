#include <cmath>

#include "TestHarness.h"

#include "Orbital/Constants.h"
#include "Orbital/MathUtil.h"
#include "ReferenceVehicles.h"
#include "Vehicle/Aerodynamics.h"
#include "Vehicle/Guidance.h"
#include "Vehicle/VehicleDef.h"

using namespace Vehicle;

namespace
{
	Orbital::FVec3 SurfacePosition(double LatitudeDegrees, double LongitudeDegrees)
	{
		const double Lat = Orbital::DegToRad(LatitudeDegrees);
		const double Lon = Orbital::DegToRad(LongitudeDegrees);
		const double Re = Orbital::Constants::Earth::EquatorialRadius;
		return {Re * std::cos(Lat) * std::cos(Lon), Re * std::cos(Lat) * std::sin(Lon), Re * std::sin(Lat)};
	}
}

OV_TEST(EngineThrustVariesWithPressure)
{
	const FEngineDef Engine = ReferenceVehicles::MakeBoosterEngine();
	const double G0 = Orbital::Constants::StandardGravity;
	const double Flow = MassFlowRate(Engine);

	OV_CHECK_NEAR(Flow, Engine.VacuumThrust / (Engine.VacuumIsp * G0), 1e-9);
	OV_CHECK_NEAR(ThrustAtPressure(Engine, 0.0), Engine.VacuumThrust, 1e-6);
	OV_CHECK_NEAR(ThrustAtPressure(Engine, 101325.0), Flow * Engine.SeaLevelIsp * G0, 1e-6);
	OV_CHECK(ThrustAtPressure(Engine, 50000.0) < Engine.VacuumThrust);
	OV_CHECK(ThrustAtPressure(Engine, 50000.0) > ThrustAtPressure(Engine, 101325.0));
}

OV_TEST(DragCoefficientPeaksTransonic)
{
	OV_CHECK(DragCoefficient(1.1) > DragCoefficient(0.5));
	OV_CHECK(DragCoefficient(1.1) > DragCoefficient(2.0));
	OV_CHECK(DragCoefficient(2.0) > DragCoefficient(5.0));
	OV_CHECK_NEAR(DragCoefficient(-1.0), DragCoefficient(0.0), 1e-12);
	OV_CHECK_NEAR(DragCoefficient(50.0), DragCoefficient(10.0), 1e-12);
}

OV_TEST(LaunchPlaneInclinations)
{
	const double CapeLatitude = 28.5623;
	const Orbital::FVec3 Cape = SurfacePosition(CapeLatitude, -80.5774);

	const FLaunchPlane DueEast = ComputeLaunchPlane(Cape, Orbital::DegToRad(CapeLatitude));
	OV_CHECK(DueEast.bInclinationReachable);
	OV_CHECK_NEAR(Orbital::RadToDeg(DueEast.Inclination), CapeLatitude, 1e-6);

	const FLaunchPlane Iss = ComputeLaunchPlane(Cape, Orbital::DegToRad(51.6));
	OV_CHECK(Iss.bInclinationReachable);
	OV_CHECK_NEAR(Orbital::RadToDeg(Iss.Inclination), 51.6, 1e-6);

	const FLaunchPlane SunSynchronous = ComputeLaunchPlane(Cape, Orbital::DegToRad(98.2));
	OV_CHECK(SunSynchronous.bInclinationReachable);
	OV_CHECK_NEAR(Orbital::RadToDeg(SunSynchronous.Inclination), 98.2, 1e-6);

	// Abaixo da latitude do sítio não há azimute que chegue lá: o plano cai para o lançamento a leste.
	const FLaunchPlane TooLow = ComputeLaunchPlane(Cape, Orbital::DegToRad(10.0));
	OV_CHECK(!TooLow.bInclinationReachable);
	OV_CHECK_NEAR(Orbital::RadToDeg(TooLow.Inclination), CapeLatitude, 1e-6);
}

OV_TEST(PitchoverKickIsClampedToItsRange)
{
	OV_CHECK_NEAR(PitchoverKickAngle(-1.0), PitchoverKickAngle(0.0), 1e-12);
	OV_CHECK_NEAR(PitchoverKickAngle(2.0), PitchoverKickAngle(1.0), 1e-12);
	OV_CHECK(PitchoverKickAngle(0.0) < PitchoverKickAngle(0.5));
	OV_CHECK(PitchoverKickAngle(0.5) < PitchoverKickAngle(1.0));
}
