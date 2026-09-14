#pragma once

#include "Orbital/OrbitalCoreApi.h"
#include "Orbital/Vec3.h"

namespace Orbital
{
	/**
	 * Propagação analítica (dois corpos) por variáveis universais: funciona para órbitas elípticas,
	 * parabólicas e hiperbólicas, com qualquer DeltaTime (positivo ou negativo). É a base do
	 * movimento "on-rails" e do time warp.
	 */
	ORBITALCORE_API FStateVector PropagateKepler(const FStateVector& Initial, double Mu, double DeltaTime);

	/** Resolve M = E - e·sin(E) para órbitas elípticas. Retorna a anomalia excêntrica E. */
	ORBITALCORE_API double SolveKeplerEquation(double MeanAnomaly, double Eccentricity);

	ORBITALCORE_API double EccentricToTrueAnomaly(double EccentricAnomaly, double Eccentricity);
	ORBITALCORE_API double TrueToEccentricAnomaly(double TrueAnomaly, double Eccentricity);
	ORBITALCORE_API double TrueToMeanAnomaly(double TrueAnomaly, double Eccentricity);
}
