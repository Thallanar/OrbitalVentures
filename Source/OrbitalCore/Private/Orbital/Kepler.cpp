#include "Orbital/Kepler.h"

#include <algorithm>
#include <cmath>

#include "Orbital/Constants.h"

namespace Orbital
{
	namespace
	{
		/** Funções de Stumpff c2 (OutC) e c3 (OutS), com série de Taylor perto de zero. */
		void ComputeStumpff(double Psi, double& OutC, double& OutS)
		{
			if (Psi > 1e-3)
			{
				const double SqrtPsi = std::sqrt(Psi);
				OutC = (1.0 - std::cos(SqrtPsi)) / Psi;
				OutS = (SqrtPsi - std::sin(SqrtPsi)) / (Psi * SqrtPsi);
			}
			else if (Psi < -1e-3)
			{
				const double SqrtNegPsi = std::sqrt(-Psi);
				OutC = (std::cosh(SqrtNegPsi) - 1.0) / -Psi;
				OutS = (std::sinh(SqrtNegPsi) - SqrtNegPsi) / (-Psi * SqrtNegPsi);
			}
			else
			{
				OutC = 1.0 / 2.0 - Psi / 24.0 + Psi * Psi / 720.0 - Psi * Psi * Psi / 40320.0;
				OutS = 1.0 / 6.0 - Psi / 120.0 + Psi * Psi / 5040.0 - Psi * Psi * Psi / 362880.0;
			}
		}
	}

	FStateVector PropagateKepler(const FStateVector& Initial, double Mu, double DeltaTime)
	{
		const FVec3& R0 = Initial.Position;
		const FVec3& V0 = Initial.Velocity;
		const double R0Mag = Length(R0);
		const double SqrtMu = std::sqrt(Mu);
		const double Sigma0 = Dot(R0, V0) / SqrtMu;
		const double Alpha = 2.0 / R0Mag - LengthSquared(V0) / Mu; // 1/a
		const bool bElliptic = Alpha * R0Mag > 1e-10;

		double Dt = DeltaTime;
		double Chi;
		if (bElliptic)
		{
			// Remove revoluções completas: mantém precisão em time warps longos.
			const double Period = Constants::TwoPi / (SqrtMu * Alpha * std::sqrt(Alpha));
			Dt = std::fmod(Dt, Period);
			Chi = SqrtMu * Alpha * Dt;
		}
		else
		{
			Chi = SqrtMu * std::abs(Alpha) * Dt;
			if (Alpha * R0Mag < -1e-10)
			{
				// Chute inicial para hipérboles (Vallado, alg. 8).
				const double A = 1.0 / Alpha;
				const double DtSign = Dt >= 0.0 ? 1.0 : -1.0;
				const double LogArg = (-2.0 * Mu * Alpha * Dt)
					/ (Dot(R0, V0) + DtSign * std::sqrt(-Mu * A) * (1.0 - R0Mag * Alpha));
				if (LogArg > 0.0 && std::isfinite(LogArg))
				{
					Chi = DtSign * std::sqrt(-A) * std::log(LogArg);
				}
			}
		}

		// Newton em Chi: dT/dChi = r, onde T = √μ·t(Chi).
		double C = 0.0;
		double S = 0.0;
		for (int Iteration = 0; Iteration < 100; ++Iteration)
		{
			const double Chi2 = Chi * Chi;
			const double Psi = Alpha * Chi2;
			ComputeStumpff(Psi, C, S);
			const double Radius = Chi2 * C + Sigma0 * Chi * (1.0 - Psi * S) + R0Mag * (1.0 - Psi * C);
			const double ScaledTime = Chi2 * Chi * S + Sigma0 * Chi2 * C + R0Mag * Chi * (1.0 - Psi * S);
			const double Step = (SqrtMu * Dt - ScaledTime) / Radius;
			Chi += Step;
			if (std::abs(Step) <= 1e-13 * std::max(1.0, std::abs(Chi)))
			{
				break;
			}
		}

		const double Chi2 = Chi * Chi;
		const double Psi = Alpha * Chi2;
		ComputeStumpff(Psi, C, S);

		const double F = 1.0 - Chi2 / R0Mag * C;
		const double G = Dt - Chi2 * Chi / SqrtMu * S;
		const FVec3 Position = F * R0 + G * V0;
		const double RMag = Length(Position);
		const double FDot = SqrtMu / (RMag * R0Mag) * Chi * (Psi * S - 1.0);
		const double GDot = 1.0 - Chi2 / RMag * C;

		return {Position, FDot * R0 + GDot * V0};
	}

	double SolveKeplerEquation(double MeanAnomaly, double Eccentricity)
	{
		// Resolve na revolução [-π, π] e devolve na mesma revolução de MeanAnomaly.
		const double Reduced = std::remainder(MeanAnomaly, Constants::TwoPi);
		double E = Reduced + (Reduced >= 0.0 ? Eccentricity : -Eccentricity);
		for (int Iteration = 0; Iteration < 50; ++Iteration)
		{
			const double Step = (E - Eccentricity * std::sin(E) - Reduced) / (1.0 - Eccentricity * std::cos(E));
			E -= Step;
			if (std::abs(Step) < 1e-15)
			{
				break;
			}
		}
		return E + (MeanAnomaly - Reduced);
	}

	double EccentricToTrueAnomaly(double EccentricAnomaly, double Eccentricity)
	{
		return 2.0 * std::atan2(
			std::sqrt(1.0 + Eccentricity) * std::sin(0.5 * EccentricAnomaly),
			std::sqrt(1.0 - Eccentricity) * std::cos(0.5 * EccentricAnomaly));
	}

	double TrueToEccentricAnomaly(double TrueAnomaly, double Eccentricity)
	{
		return 2.0 * std::atan2(
			std::sqrt(1.0 - Eccentricity) * std::sin(0.5 * TrueAnomaly),
			std::sqrt(1.0 + Eccentricity) * std::cos(0.5 * TrueAnomaly));
	}

	double TrueToMeanAnomaly(double TrueAnomaly, double Eccentricity)
	{
		const double E = TrueToEccentricAnomaly(TrueAnomaly, Eccentricity);
		return E - Eccentricity * std::sin(E);
	}
}
