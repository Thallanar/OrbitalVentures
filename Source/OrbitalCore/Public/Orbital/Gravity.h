#pragma once

#include "Orbital/OrbitalCoreApi.h"
#include "Orbital/Vec3.h"

namespace Orbital
{
	ORBITALCORE_API FVec3 PointMassAcceleration(const FVec3& Position, double Mu);

	/** Perturbação do achatamento (J2), em coordenadas com Z no eixo de rotação do corpo. */
	ORBITALCORE_API FVec3 J2Acceleration(const FVec3& Position, double Mu, double J2, double EquatorialRadius);

	/** Taxa secular de precessão do nodo ascendente por J2 (rad/s). */
	ORBITALCORE_API double NodalPrecessionRate(
		double SemiMajorAxis, double Eccentricity, double Inclination, double Mu, double J2, double EquatorialRadius);

	/** Inclinação heliossíncrona (rad) para a Terra; NaN se a órbita for alta demais para existir. */
	ORBITALCORE_API double SunSynchronousInclination(double SemiMajorAxis, double Eccentricity);

	/** Raio da esfera de influência (Laplace) de um corpo menor orbitando um maior. */
	ORBITALCORE_API double SphereOfInfluenceRadius(double SemiMajorAxis, double MuSmall, double MuLarge);
}
