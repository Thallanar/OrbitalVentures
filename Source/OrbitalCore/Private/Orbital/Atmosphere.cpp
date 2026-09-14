#include "Orbital/Atmosphere.h"

#include <cmath>

namespace Orbital
{
	namespace
	{
		constexpr double StdAtmEarthRadius = 6356766.0;         // m, raio usado na altitude geopotencial
		constexpr double StdAtmGasConstant = 287.0531;          // J/(kg·K), ar seco
		constexpr double StdAtmHydrostaticConstant = 0.034163195; // g0·M/R*, K/m
		constexpr double StdAtmHeatRatio = 1.4;
		constexpr double StdAtmUpperLimit = 86000.0;            // m geométricos
		constexpr double StdAtmTemperatureAt86Km = 186.946;     // K

		struct FStdAtmLayer
		{
			double BaseGeopotentialAltitude; // m
			double LapseRate;                // K/m
			double BaseTemperature;          // K
			double BasePressure;             // Pa
		};

		constexpr FStdAtmLayer StdAtmLayers[] = {
			{0.0, -0.0065, 288.15, 101325.0},
			{11000.0, 0.0, 216.65, 22632.06},
			{20000.0, 0.001, 216.65, 5474.889},
			{32000.0, 0.0028, 228.65, 868.0187},
			{47000.0, 0.0, 270.65, 110.9063},
			{51000.0, -0.0028, 270.65, 66.93887},
			{71000.0, -0.002, 214.65, 3.956420},
		};

		struct FExponentialDensityBand
		{
			double BaseAltitude; // m
			double BaseDensity;  // kg/m³
			double ScaleHeight;  // m
		};

		// Vallado, "Fundamentals of Astrodynamics and Applications", tabela 8-4 (a partir de 80 km).
		constexpr FExponentialDensityBand UpperAtmosphereBands[] = {
			{80e3, 1.905e-5, 5.799e3},
			{90e3, 3.396e-6, 5.382e3},
			{100e3, 5.297e-7, 5.877e3},
			{110e3, 9.661e-8, 7.263e3},
			{120e3, 2.438e-8, 9.473e3},
			{130e3, 8.484e-9, 12.636e3},
			{140e3, 3.845e-9, 16.149e3},
			{150e3, 2.070e-9, 22.523e3},
			{180e3, 5.464e-10, 29.740e3},
			{200e3, 2.789e-10, 37.105e3},
			{250e3, 7.248e-11, 45.546e3},
			{300e3, 2.418e-11, 53.628e3},
			{350e3, 9.518e-12, 53.298e3},
			{400e3, 3.725e-12, 58.515e3},
			{450e3, 1.585e-12, 60.828e3},
			{500e3, 6.967e-13, 63.822e3},
			{600e3, 1.454e-13, 71.835e3},
			{700e3, 3.614e-14, 88.667e3},
			{800e3, 1.170e-14, 124.64e3},
			{900e3, 5.245e-15, 181.05e3},
			{1000e3, 3.019e-15, 268.00e3},
		};

		double SpeedOfSoundAt(double Temperature)
		{
			return std::sqrt(StdAtmHeatRatio * StdAtmGasConstant * Temperature);
		}

		FAtmosphereSample SampleStandardAtmosphere(double GeometricAltitude)
		{
			const double Geopotential = StdAtmEarthRadius * GeometricAltitude / (StdAtmEarthRadius + GeometricAltitude);

			const FStdAtmLayer* Layer = &StdAtmLayers[0];
			for (const FStdAtmLayer& Candidate : StdAtmLayers)
			{
				if (Geopotential >= Candidate.BaseGeopotentialAltitude)
				{
					Layer = &Candidate;
				}
			}

			const double DeltaH = Geopotential - Layer->BaseGeopotentialAltitude;
			FAtmosphereSample Out;
			Out.Temperature = Layer->BaseTemperature + Layer->LapseRate * DeltaH;
			Out.Pressure = Layer->LapseRate == 0.0
				? Layer->BasePressure * std::exp(-StdAtmHydrostaticConstant * DeltaH / Layer->BaseTemperature)
				: Layer->BasePressure * std::pow(Layer->BaseTemperature / Out.Temperature, StdAtmHydrostaticConstant / Layer->LapseRate);
			Out.Density = Out.Pressure / (StdAtmGasConstant * Out.Temperature);
			Out.SpeedOfSound = SpeedOfSoundAt(Out.Temperature);
			return Out;
		}
	}

	FAtmosphereSample SampleEarthAtmosphere(double GeometricAltitude)
	{
		if (GeometricAltitude < StdAtmUpperLimit)
		{
			return SampleStandardAtmosphere(GeometricAltitude);
		}

		const FExponentialDensityBand* Band = &UpperAtmosphereBands[0];
		for (const FExponentialDensityBand& Candidate : UpperAtmosphereBands)
		{
			if (GeometricAltitude >= Candidate.BaseAltitude)
			{
				Band = &Candidate;
			}
		}

		FAtmosphereSample Out;
		Out.Temperature = StdAtmTemperatureAt86Km;
		Out.Density = Band->BaseDensity * std::exp(-(GeometricAltitude - Band->BaseAltitude) / Band->ScaleHeight);
		Out.Pressure = Out.Density * StdAtmGasConstant * Out.Temperature;
		Out.SpeedOfSound = SpeedOfSoundAt(Out.Temperature);
		return Out;
	}
}
