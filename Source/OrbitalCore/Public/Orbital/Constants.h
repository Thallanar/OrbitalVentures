#pragma once

// Unidades SI em todo o OrbitalCore: metros, segundos, quilogramas, radianos.
namespace Orbital::Constants
{
	inline constexpr double Pi = 3.14159265358979323846;
	inline constexpr double TwoPi = 2.0 * Pi;
	inline constexpr double StandardGravity = 9.80665;             // m/s²
	inline constexpr double AstronomicalUnit = 1.495978707e11;     // m
	inline constexpr double TropicalYear = 365.24219 * 86400.0;    // s

	namespace Sun
	{
		inline constexpr double Mu = 1.32712440018e20;             // m³/s²
	}

	namespace Earth
	{
		inline constexpr double Mu = 3.986004418e14;               // m³/s² (WGS84)
		inline constexpr double EquatorialRadius = 6378137.0;      // m (WGS84)
		inline constexpr double Flattening = 1.0 / 298.257223563;  // WGS84
		inline constexpr double PolarRadius = EquatorialRadius * (1.0 - Flattening);
		inline constexpr double EccentricitySquared = Flattening * (2.0 - Flattening);
		inline constexpr double J2 = 1.08262668e-3;
		inline constexpr double SiderealDay = 86164.0905;          // s
		inline constexpr double RotationRate = 7.292115e-5;        // rad/s
	}

	namespace Moon
	{
		inline constexpr double Mu = 4.9048695e12;                 // m³/s²
		inline constexpr double EquatorialRadius = 1738.1e3;       // m
		inline constexpr double SemiMajorAxis = 384399e3;          // m (em torno da Terra)
	}
}
