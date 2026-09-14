#pragma once

#include "Orbital/OrbitalCoreApi.h"
#include "Orbital/Vec3.h"

namespace Orbital
{
	/**
	 * Elementos keplerianos clássicos. Ângulos em radianos, distâncias em metros.
	 * Hiperbólicas têm SemiMajorAxis < 0. Parabólicas exatas (e == 1) não são suportadas.
	 *
	 * Convenções para casos degenerados:
	 *  - circular:            ArgumentOfPeriapsis = 0, TrueAnomaly = argumento da latitude;
	 *  - equatorial:          RightAscensionOfAscendingNode = 0, ArgumentOfPeriapsis = longitude do periapse;
	 *  - circular equatorial: TrueAnomaly = longitude verdadeira.
	 */
	struct FOrbitalElements
	{
		double SemiMajorAxis = 0.0;
		double Eccentricity = 0.0;
		double Inclination = 0.0;
		double RightAscensionOfAscendingNode = 0.0;
		double ArgumentOfPeriapsis = 0.0;
		double TrueAnomaly = 0.0;
	};

	ORBITALCORE_API FOrbitalElements StateToElements(const FStateVector& State, double Mu);
	ORBITALCORE_API FStateVector ElementsToState(const FOrbitalElements& Elements, double Mu);

	/** Período em segundos; infinito para órbitas abertas. */
	ORBITALCORE_API double OrbitalPeriod(double SemiMajorAxis, double Mu);
	ORBITALCORE_API double SpecificOrbitalEnergy(const FStateVector& State, double Mu);
	ORBITALCORE_API double CircularOrbitSpeed(double Radius, double Mu);
	ORBITALCORE_API double PeriapsisRadius(const FOrbitalElements& Elements);
	/** Infinito para órbitas abertas. */
	ORBITALCORE_API double ApoapsisRadius(const FOrbitalElements& Elements);
}
