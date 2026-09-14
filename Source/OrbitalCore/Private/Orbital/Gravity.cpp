#include "Orbital/Gravity.h"

#include <cmath>
#include <limits>

#include "Orbital/Constants.h"

namespace Orbital
{
	FVec3 PointMassAcceleration(const FVec3& Position, double Mu)
	{
		const double R2 = LengthSquared(Position);
		return Position * (-Mu / (R2 * std::sqrt(R2)));
	}

	FVec3 J2Acceleration(const FVec3& Position, double Mu, double J2, double EquatorialRadius)
	{
		const double R2 = LengthSquared(Position);
		const double R = std::sqrt(R2);
		const double Factor = -1.5 * J2 * Mu * EquatorialRadius * EquatorialRadius / (R2 * R2 * R);
		const double FiveZ2OverR2 = 5.0 * Position.Z * Position.Z / R2;
		return FVec3(
			Position.X * (1.0 - FiveZ2OverR2),
			Position.Y * (1.0 - FiveZ2OverR2),
			Position.Z * (3.0 - FiveZ2OverR2)) * Factor;
	}

	double NodalPrecessionRate(
		double SemiMajorAxis, double Eccentricity, double Inclination, double Mu, double J2, double EquatorialRadius)
	{
		const double MeanMotion = std::sqrt(Mu / (SemiMajorAxis * SemiMajorAxis * SemiMajorAxis));
		const double SemiLatusRectum = SemiMajorAxis * (1.0 - Eccentricity * Eccentricity);
		const double RadiusRatio = EquatorialRadius / SemiLatusRectum;
		return -1.5 * MeanMotion * J2 * RadiusRatio * RadiusRatio * std::cos(Inclination);
	}

	double SunSynchronousInclination(double SemiMajorAxis, double Eccentricity)
	{
		using namespace Constants;
		// Precessão necessária: uma volta por ano tropical, acompanhando o Sol.
		const double RequiredRate = TwoPi / TropicalYear;
		const double RateAtZeroInclination =
			NodalPrecessionRate(SemiMajorAxis, Eccentricity, 0.0, Earth::Mu, Earth::J2, Earth::EquatorialRadius);
		const double CosInclination = RequiredRate / RateAtZeroInclination;
		if (CosInclination < -1.0 || CosInclination > 1.0)
		{
			return std::numeric_limits<double>::quiet_NaN();
		}
		return std::acos(CosInclination);
	}

	double SphereOfInfluenceRadius(double SemiMajorAxis, double MuSmall, double MuLarge)
	{
		return SemiMajorAxis * std::pow(MuSmall / MuLarge, 0.4);
	}
}
