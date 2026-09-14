#include "Orbital/Maneuvers.h"

#include <cmath>

#include "Orbital/Constants.h"

namespace Orbital
{
	FHohmannTransfer ComputeHohmannTransfer(double R1, double R2, double Mu)
	{
		const double SumRadii = R1 + R2;
		const double TransferSemiMajorAxis = 0.5 * SumRadii;

		FHohmannTransfer Out;
		Out.DeltaV1 = std::abs(std::sqrt(Mu / R1) * (std::sqrt(2.0 * R2 / SumRadii) - 1.0));
		Out.DeltaV2 = std::abs(std::sqrt(Mu / R2) * (1.0 - std::sqrt(2.0 * R1 / SumRadii)));
		Out.TotalDeltaV = Out.DeltaV1 + Out.DeltaV2;
		Out.TransferTime = Constants::Pi
			* std::sqrt(TransferSemiMajorAxis * TransferSemiMajorAxis * TransferSemiMajorAxis / Mu);
		return Out;
	}

	double PlaneChangeDeltaV(double Speed, double DeltaInclination)
	{
		return 2.0 * Speed * std::sin(0.5 * std::abs(DeltaInclination));
	}
}
