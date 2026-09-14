#pragma once

#include "Orbital/OrbitalCoreApi.h"

namespace Orbital
{
	struct FHohmannTransfer
	{
		double DeltaV1 = 0.0;      // m/s, na órbita inicial
		double DeltaV2 = 0.0;      // m/s, na circularização
		double TotalDeltaV = 0.0;  // m/s
		double TransferTime = 0.0; // s, meia elipse de transferência
	};

	/** Transferência de Hohmann entre órbitas circulares coplanares de raios R1 e R2. */
	ORBITALCORE_API FHohmannTransfer ComputeHohmannTransfer(double R1, double R2, double Mu);

	/** Δv de uma mudança de plano pura com velocidade Speed. */
	ORBITALCORE_API double PlaneChangeDeltaV(double Speed, double DeltaInclination);
}
