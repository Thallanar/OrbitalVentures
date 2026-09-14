#include "Orbital/OrbitalElements.h"

#include <cmath>
#include <limits>

#include "Orbital/MathUtil.h"

namespace Orbital
{
	namespace
	{
		constexpr double DegenerateOrbitEpsilon = 1e-11;
	}

	FOrbitalElements StateToElements(const FStateVector& State, double Mu)
	{
		const FVec3& R = State.Position;
		const FVec3& V = State.Velocity;
		const double RMag = Length(R);
		const double RDotV = Dot(R, V);

		const FVec3 H = Cross(R, V);
		const double HMag = Length(H);
		const FVec3 NodeLine(-H.Y, H.X, 0.0);
		const double NodeMag = Length(NodeLine);
		const FVec3 EccVec = ((LengthSquared(V) - Mu / RMag) * R - RDotV * V) / Mu;
		const double Ecc = Length(EccVec);
		const double Energy = SpecificOrbitalEnergy(State, Mu);

		FOrbitalElements Out;
		Out.Eccentricity = Ecc;
		Out.SemiMajorAxis = Energy != 0.0 ? -Mu / (2.0 * Energy) : std::numeric_limits<double>::infinity();
		Out.Inclination = std::atan2(std::sqrt(H.X * H.X + H.Y * H.Y), H.Z);

		const bool bCircular = Ecc < DegenerateOrbitEpsilon;
		const bool bEquatorial = NodeMag < DegenerateOrbitEpsilon * HMag;
		// Em órbitas equatoriais os ângulos são medidos a partir do eixo X, no sentido do movimento.
		const double EquatorialSign = H.Z >= 0.0 ? 1.0 : -1.0;

		if (!bEquatorial)
		{
			Out.RightAscensionOfAscendingNode = WrapTwoPi(std::atan2(NodeLine.Y, NodeLine.X));
		}

		// Ângulos no plano orbital: atan2((A×B)·Ĥ, A·B) mede de A para B no sentido do movimento.
		if (!bCircular)
		{
			Out.ArgumentOfPeriapsis = bEquatorial
				? WrapTwoPi(std::atan2(EquatorialSign * EccVec.Y, EccVec.X))
				: WrapTwoPi(std::atan2(Dot(Cross(NodeLine, EccVec), H) / HMag, Dot(NodeLine, EccVec)));
			Out.TrueAnomaly = WrapTwoPi(std::atan2(Dot(Cross(EccVec, R), H) / HMag, Dot(EccVec, R)));
		}
		else if (!bEquatorial)
		{
			Out.TrueAnomaly = WrapTwoPi(std::atan2(Dot(Cross(NodeLine, R), H) / HMag, Dot(NodeLine, R)));
		}
		else
		{
			Out.TrueAnomaly = WrapTwoPi(std::atan2(EquatorialSign * R.Y, R.X));
		}

		return Out;
	}

	FStateVector ElementsToState(const FOrbitalElements& Elements, double Mu)
	{
		const double Ecc = Elements.Eccentricity;
		const double SemiLatusRectum = Elements.SemiMajorAxis * (1.0 - Ecc * Ecc);
		const double CosNu = std::cos(Elements.TrueAnomaly);
		const double SinNu = std::sin(Elements.TrueAnomaly);
		const double RMag = SemiLatusRectum / (1.0 + Ecc * CosNu);
		const double SpeedScale = std::sqrt(Mu / SemiLatusRectum);

		// Base perifocal (P aponta para o periapse, Q a 90° no sentido do movimento) no referencial inercial.
		const double CosRaan = std::cos(Elements.RightAscensionOfAscendingNode);
		const double SinRaan = std::sin(Elements.RightAscensionOfAscendingNode);
		const double CosInc = std::cos(Elements.Inclination);
		const double SinInc = std::sin(Elements.Inclination);
		const double CosArgP = std::cos(Elements.ArgumentOfPeriapsis);
		const double SinArgP = std::sin(Elements.ArgumentOfPeriapsis);

		const FVec3 PHat(
			CosRaan * CosArgP - SinRaan * SinArgP * CosInc,
			SinRaan * CosArgP + CosRaan * SinArgP * CosInc,
			SinArgP * SinInc);
		const FVec3 QHat(
			-CosRaan * SinArgP - SinRaan * CosArgP * CosInc,
			-SinRaan * SinArgP + CosRaan * CosArgP * CosInc,
			CosArgP * SinInc);

		return {
			(RMag * CosNu) * PHat + (RMag * SinNu) * QHat,
			(-SpeedScale * SinNu) * PHat + (SpeedScale * (Ecc + CosNu)) * QHat};
	}

	double OrbitalPeriod(double SemiMajorAxis, double Mu)
	{
		if (SemiMajorAxis <= 0.0)
		{
			return std::numeric_limits<double>::infinity();
		}
		return Constants::TwoPi * std::sqrt(SemiMajorAxis * SemiMajorAxis * SemiMajorAxis / Mu);
	}

	double SpecificOrbitalEnergy(const FStateVector& State, double Mu)
	{
		return 0.5 * LengthSquared(State.Velocity) - Mu / Length(State.Position);
	}

	double CircularOrbitSpeed(double Radius, double Mu)
	{
		return std::sqrt(Mu / Radius);
	}

	double PeriapsisRadius(const FOrbitalElements& Elements)
	{
		return Elements.SemiMajorAxis * (1.0 - Elements.Eccentricity);
	}

	double ApoapsisRadius(const FOrbitalElements& Elements)
	{
		if (Elements.Eccentricity >= 1.0)
		{
			return std::numeric_limits<double>::infinity();
		}
		return Elements.SemiMajorAxis * (1.0 + Elements.Eccentricity);
	}
}
