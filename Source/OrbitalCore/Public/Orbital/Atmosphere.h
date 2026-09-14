#pragma once

#include "Orbital/OrbitalCoreApi.h"

namespace Orbital
{
	struct FAtmosphereSample
	{
		double Temperature = 0.0;  // K
		double Pressure = 0.0;     // Pa
		double Density = 0.0;      // kg/m³
		double SpeedOfSound = 0.0; // m/s
	};

	/**
	 * Atmosfera terrestre por altitude geométrica (m).
	 * Até 86 km: US Standard Atmosphere 1976. Acima: densidade por tabela exponencial (Vallado),
	 * com temperatura fixa — suficiente para arrasto e decaimento orbital.
	 */
	ORBITALCORE_API FAtmosphereSample SampleEarthAtmosphere(double GeometricAltitude);
}
