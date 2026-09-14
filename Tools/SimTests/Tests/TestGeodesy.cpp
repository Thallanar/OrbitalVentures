#include <cmath>

#include "TestHarness.h"

#include "Orbital/Constants.h"
#include "Orbital/Geodesy.h"
#include "Orbital/MathUtil.h"

using namespace Orbital;

OV_TEST(Wgs84DerivedConstants)
{
	// Valores publicados na especificação do WGS84 (NIMA TR8350.2).
	OV_CHECK_NEAR(Constants::Earth::PolarRadius, 6356752.3142, 1e-4);
	OV_CHECK_NEAR(Constants::Earth::EccentricitySquared, 6.69437999014e-3, 1e-14);
}

OV_TEST(GeodeticToCartesianAtReferencePoints)
{
	const FVec3 Equator = GeodeticToCartesian({0.0, 0.0, 0.0});
	OV_CHECK_NEAR(Equator.X, Constants::Earth::EquatorialRadius, 1e-6);
	OV_CHECK_NEAR(Equator.Y, 0.0, 1e-6);
	OV_CHECK_NEAR(Equator.Z, 0.0, 1e-6);

	const FVec3 NorthPole = GeodeticToCartesian({DegToRad(90.0), 0.0, 0.0});
	OV_CHECK_NEAR(Length(FVec3(NorthPole.X, NorthPole.Y, 0.0)), 0.0, 1e-6);
	OV_CHECK_NEAR(NorthPole.Z, Constants::Earth::PolarRadius, 1e-6);

	const FVec3 Elevated = GeodeticToCartesian({0.0, DegToRad(90.0), 1000.0});
	OV_CHECK_NEAR(Elevated.Y, Constants::Earth::EquatorialRadius + 1000.0, 1e-6);
}

OV_TEST(GeodeticRoundTrip)
{
	for (double LatDeg = -90.0; LatDeg <= 90.0; LatDeg += 7.5)
	{
		for (double LonDeg = -180.0; LonDeg < 180.0; LonDeg += 45.0)
		{
			for (const double Height : {-500.0, 0.0, 1000.0, 400e3, 35786e3})
			{
				const FGeodeticCoordinates In{DegToRad(LatDeg), DegToRad(LonDeg), Height};
				const FVec3 Position = GeodeticToCartesian(In);
				const FGeodeticCoordinates Out = CartesianToGeodetic(Position);
				OV_CHECK_NEAR(Out.Latitude, In.Latitude, 1e-11);
				OV_CHECK_NEAR(Out.Height, In.Height, 1e-4);
				OV_CHECK(Length(GeodeticToCartesian(Out) - Position) < 1e-4);
				if (std::abs(LatDeg) < 90.0)
				{
					OV_CHECK(std::abs(std::remainder(Out.Longitude - In.Longitude, Constants::TwoPi)) < 1e-12);
				}
			}
		}
	}
}

OV_TEST(GeodeticLatitudeIsSteeperThanGeocentric)
{
	// Identidade clássica na superfície: tan(geocêntrica) = (1 − e²)·tan(geodésica).
	const double Geodetic = DegToRad(45.0);
	const FVec3 Surface = GeodeticToCartesian({Geodetic, 0.0, 0.0});
	const double Geocentric = std::atan2(Surface.Z, Surface.X);
	OV_CHECK_NEAR(Geocentric, std::atan((1.0 - Constants::Earth::EccentricitySquared) * std::tan(Geodetic)), 1e-12);
	OV_CHECK_NEAR(RadToDeg(Geodetic - Geocentric), 0.1924, 1e-4);
}

OV_TEST(GeodeticUpIsNormalToTheSurface)
{
	for (const double LatDeg : {0.0, 28.5623, 45.0, 70.0})
	{
		const FGeodeticCoordinates Point{DegToRad(LatDeg), DegToRad(-80.0), 0.0};
		const FVec3 Here = GeodeticToCartesian(Point);
		const FVec3 Up = GeodeticUp(Here);
		OV_CHECK_NEAR(Length(Up), 1.0, 1e-12);

		// Deslocamentos pequenos sobre o elipsoide, ao longo do meridiano e do paralelo, ficam no plano tangente.
		// Cada deslocamento é uma corda, que se afasta da tangente por metade do ângulo do passo (~5e-7). Uma
		// vertical geocêntrica no lugar da geodésica erraria ~3e-3 no meridiano fora do equador: 1e-5 separa os dois.
		const double Step = 1e-6;
		const FVec3 AlongMeridian = GeodeticToCartesian({Point.Latitude + Step, Point.Longitude, 0.0}) - Here;
		const FVec3 AlongParallel = GeodeticToCartesian({Point.Latitude, Point.Longitude + Step, 0.0}) - Here;
		OV_CHECK(std::abs(Dot(Normalized(AlongMeridian), Up)) < 1e-5);
		OV_CHECK(std::abs(Dot(Normalized(AlongParallel), Up)) < 1e-5);
	}
}
