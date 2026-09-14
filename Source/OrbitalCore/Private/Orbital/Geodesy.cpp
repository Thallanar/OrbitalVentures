#include "Orbital/Geodesy.h"

#include <algorithm>
#include <cmath>

#include "Orbital/Constants.h"

namespace Orbital
{
	namespace
	{
		constexpr double GeodesyA = Constants::Earth::EquatorialRadius;
		constexpr double GeodesyB = Constants::Earth::PolarRadius;
		constexpr double GeodesyE2 = Constants::Earth::EccentricitySquared;
	}

	FVec3 GeodeticToCartesian(const FGeodeticCoordinates& Coordinates)
	{
		const double SinLat = std::sin(Coordinates.Latitude);
		const double CosLat = std::cos(Coordinates.Latitude);
		// Raio de curvatura no primeiro vertical.
		const double N = GeodesyA / std::sqrt(1.0 - GeodesyE2 * SinLat * SinLat);
		return {
			(N + Coordinates.Height) * CosLat * std::cos(Coordinates.Longitude),
			(N + Coordinates.Height) * CosLat * std::sin(Coordinates.Longitude),
			(N * (1.0 - GeodesyE2) + Coordinates.Height) * SinLat};
	}

	FGeodeticCoordinates CartesianToGeodetic(const FVec3& Position)
	{
		const double A2 = GeodesyA * GeodesyA;
		const double B2 = GeodesyB * GeodesyB;
		const double SecondE2 = (A2 - B2) / B2;
		const double Z = Position.Z;
		const double Z2 = Z * Z;
		const double P2 = Position.X * Position.X + Position.Y * Position.Y;
		const double P = std::sqrt(P2);

		FGeodeticCoordinates Out;
		Out.Longitude = std::atan2(Position.Y, Position.X);
		if (P2 + Z2 == 0.0)
		{
			Out.Height = -GeodesyA;
			return Out;
		}

		const double F = 54.0 * B2 * Z2;
		const double G = P2 + (1.0 - GeodesyE2) * Z2 - GeodesyE2 * (A2 - B2);
		const double C = GeodesyE2 * GeodesyE2 * F * P2 / (G * G * G);
		const double S = std::cbrt(1.0 + C + std::sqrt(C * C + 2.0 * C));
		const double K = S + 1.0 + 1.0 / S;
		const double PFactor = F / (3.0 * K * K * G * G);
		const double Q = std::sqrt(1.0 + 2.0 * GeodesyE2 * GeodesyE2 * PFactor);
		// Nos polos o termo sob a raiz é zero e pode sair levemente negativo por arredondamento.
		const double R0 = -(PFactor * GeodesyE2 * P) / (1.0 + Q)
			+ std::sqrt(std::max(0.0, 0.5 * A2 * (1.0 + 1.0 / Q)
				- PFactor * (1.0 - GeodesyE2) * Z2 / (Q * (1.0 + Q)) - 0.5 * PFactor * P2));
		const double Offset = P - GeodesyE2 * R0;
		const double U = std::sqrt(Offset * Offset + Z2);
		const double V = std::sqrt(Offset * Offset + (1.0 - GeodesyE2) * Z2);
		const double Z0 = B2 * Z / (GeodesyA * V);

		Out.Height = U * (1.0 - B2 / (GeodesyA * V));
		Out.Latitude = std::atan2(Z + SecondE2 * Z0, P);
		return Out;
	}

	double GeodeticHeight(const FVec3& Position)
	{
		return CartesianToGeodetic(Position).Height;
	}

	FVec3 GeodeticUp(const FVec3& Position)
	{
		const FGeodeticCoordinates Coordinates = CartesianToGeodetic(Position);
		const double CosLat = std::cos(Coordinates.Latitude);
		return {CosLat * std::cos(Coordinates.Longitude), CosLat * std::sin(Coordinates.Longitude), std::sin(Coordinates.Latitude)};
	}
}
