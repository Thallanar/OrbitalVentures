#include "TestHarness.h"

#include "Orbital/Atmosphere.h"

using namespace Orbital;

// Valores de referência: tabelas da US Standard Atmosphere 1976 (altitude geométrica).
OV_TEST(StandardAtmosphereSeaLevel)
{
	const FAtmosphereSample S = SampleEarthAtmosphere(0.0);
	OV_CHECK_NEAR(S.Temperature, 288.15, 1e-9);
	OV_CHECK_NEAR(S.Pressure, 101325.0, 1e-6);
	OV_CHECK_NEAR(S.Density, 1.2250, 1e-4);
	OV_CHECK_NEAR(S.SpeedOfSound, 340.29, 0.05);
}

OV_TEST(StandardAtmosphereTableValues)
{
	const FAtmosphereSample At10Km = SampleEarthAtmosphere(10000.0);
	OV_CHECK_NEAR(At10Km.Temperature, 223.252, 0.01);
	OV_CHECK_NEAR(At10Km.Pressure, 26499.9, 10.0);
	OV_CHECK_NEAR(At10Km.Density, 0.41351, 5e-4);

	const FAtmosphereSample At20Km = SampleEarthAtmosphere(20000.0);
	OV_CHECK_NEAR(At20Km.Temperature, 216.65, 0.01);
	OV_CHECK_NEAR(At20Km.Pressure, 5529.3, 5.0);
	OV_CHECK_NEAR(At20Km.Density, 0.088910, 1e-4);

	const FAtmosphereSample At50Km = SampleEarthAtmosphere(50000.0);
	OV_CHECK_NEAR(At50Km.Temperature, 270.65, 0.01);
	OV_CHECK_NEAR(At50Km.Pressure, 79.779, 0.5);
	OV_CHECK_NEAR(At50Km.Density, 1.0269e-3, 1e-5);
}

OV_TEST(UpperAtmosphereIsContinuousAndDecreasing)
{
	const double Below = SampleEarthAtmosphere(85999.0).Density;
	const double Above = SampleEarthAtmosphere(86000.0).Density;
	OV_CHECK(std::abs(Above / Below - 1.0) < 0.05);

	const double At400Km = SampleEarthAtmosphere(400e3).Density;
	OV_CHECK(At400Km > 1e-12 && At400Km < 1e-11);

	double Previous = SampleEarthAtmosphere(0.0).Density;
	for (double Altitude = 1000.0; Altitude <= 2000e3; Altitude += 1000.0)
	{
		const double Current = SampleEarthAtmosphere(Altitude).Density;
		OV_CHECK(Current < Previous);
		Previous = Current;
	}
}
