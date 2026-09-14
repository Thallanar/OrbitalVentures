#pragma once

#include <cmath>

namespace Orbital
{
	/** Vetor 3D em double. Independente do FVector do UE para o núcleo compilar fora do motor. */
	struct FVec3
	{
		double X = 0.0;
		double Y = 0.0;
		double Z = 0.0;

		constexpr FVec3() = default;
		constexpr FVec3(double InX, double InY, double InZ) : X(InX), Y(InY), Z(InZ) {}

		constexpr FVec3 operator+(const FVec3& Other) const { return {X + Other.X, Y + Other.Y, Z + Other.Z}; }
		constexpr FVec3 operator-(const FVec3& Other) const { return {X - Other.X, Y - Other.Y, Z - Other.Z}; }
		constexpr FVec3 operator-() const { return {-X, -Y, -Z}; }
		constexpr FVec3 operator*(double Scale) const { return {X * Scale, Y * Scale, Z * Scale}; }
		constexpr FVec3 operator/(double Scale) const { return {X / Scale, Y / Scale, Z / Scale}; }

		constexpr FVec3& operator+=(const FVec3& Other) { X += Other.X; Y += Other.Y; Z += Other.Z; return *this; }
		constexpr FVec3& operator-=(const FVec3& Other) { X -= Other.X; Y -= Other.Y; Z -= Other.Z; return *this; }
	};

	constexpr FVec3 operator*(double Scale, const FVec3& V) { return V * Scale; }

	constexpr double Dot(const FVec3& A, const FVec3& B) { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }

	constexpr FVec3 Cross(const FVec3& A, const FVec3& B)
	{
		return {A.Y * B.Z - A.Z * B.Y, A.Z * B.X - A.X * B.Z, A.X * B.Y - A.Y * B.X};
	}

	constexpr double LengthSquared(const FVec3& V) { return Dot(V, V); }
	inline double Length(const FVec3& V) { return std::sqrt(LengthSquared(V)); }

	inline FVec3 Normalized(const FVec3& V)
	{
		const double Len = Length(V);
		return Len > 0.0 ? V / Len : FVec3();
	}

	/** Posição (m) e velocidade (m/s) num referencial inercial centrado no corpo dominante. */
	struct FStateVector
	{
		FVec3 Position;
		FVec3 Velocity;
	};
}
