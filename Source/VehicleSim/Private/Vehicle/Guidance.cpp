#include "Vehicle/Guidance.h"

#include <algorithm>
#include <cmath>

#include "Orbital/MathUtil.h"

namespace Vehicle
{
	namespace
	{
		constexpr double GuidanceMinTimeToGo = 1.0;         // s, evita divisão por quase zero no fim da queima
		constexpr double GuidanceMaxVerticalFraction = 0.95; // o empuxo nunca aponta totalmente para cima ou para baixo
		constexpr double GuidanceMaxCrossFraction = 0.3;     // correção lateral limitada
		constexpr double GuidanceMinKickDegrees = 5.0;
		constexpr double GuidanceMaxKickDegrees = 20.0;
	}

	FLaunchPlane ComputeLaunchPlane(const Orbital::FVec3& SitePosition, double TargetInclination)
	{
		// O plano orbital passa pelo centro da Terra, então aqui a vertical e a latitude são geocêntricas: a menor
		// inclinação alcançável é a latitude geocêntrica do sítio, um pouco menor que a geodésica.
		const Orbital::FVec3 Up = Orbital::Normalized(SitePosition);
		const Orbital::FVec3 East = Orbital::Normalized(Orbital::FVec3(-Up.Y, Up.X, 0.0));
		const Orbital::FVec3 North = Orbital::Cross(Up, East);
		const double Latitude = std::asin(Up.Z);

		// Azimute inercial (a partir do norte): sin(Az) = cos(i) / cos(latitude).
		const double Ratio = std::cos(TargetInclination) / std::cos(Latitude);

		FLaunchPlane Out;
		Out.bInclinationReachable = std::abs(Ratio) <= 1.0 + 1e-12;
		const double Azimuth = std::asin(std::clamp(Ratio, -1.0, 1.0));
		const Orbital::FVec3 LaunchDirection = East * std::sin(Azimuth) + North * std::cos(Azimuth);
		Out.Normal = Orbital::Normalized(Orbital::Cross(Up, LaunchDirection));
		Out.Inclination = std::acos(std::clamp(Out.Normal.Z, -1.0, 1.0));
		return Out;
	}

	double PitchoverKickAngle(double Aggressiveness)
	{
		const double Clamped = std::clamp(Aggressiveness, 0.0, 1.0);
		return Orbital::DegToRad(GuidanceMinKickDegrees + (GuidanceMaxKickDegrees - GuidanceMinKickDegrees) * Clamped);
	}

	FClosedLoopCommand ComputeClosedLoopCommand(const FClosedLoopInput& Input)
	{
		const Orbital::FVec3& Position = Input.State.Position;
		const Orbital::FVec3& Velocity = Input.State.Velocity;
		const double Radius = Orbital::Length(Position);
		const Orbital::FVec3 Up = Position / Radius;
		const Orbital::FVec3 Downrange = Orbital::Normalized(Orbital::Cross(Input.PlaneNormal, Up));

		const double VerticalSpeed = Orbital::Dot(Velocity, Up);
		const double HorizontalSpeed = Orbital::Dot(Velocity, Downrange);
		const double CrossSpeed = Orbital::Dot(Velocity, Input.PlaneNormal);
		const double CrossOffset = Orbital::Dot(Position, Input.PlaneNormal);

		// Tempo até o corte pela equação do foguete, usando o Δv que ainda falta.
		const double TargetSpeed = std::sqrt(Input.Mu / Input.TargetRadius);
		const double MissingSpeed = TargetSpeed - HorizontalSpeed;
		const double DeltaVToGo = std::sqrt(MissingSpeed * MissingSpeed + VerticalSpeed * VerticalSpeed + CrossSpeed * CrossSpeed);
		const double TimeToGo = std::max(GuidanceMinTimeToGo,
			(Input.ExhaustVelocity / Input.ThrustAcceleration) * (1.0 - std::exp(-DeltaVToGo / Input.ExhaustVelocity)));

		// Aceleração com perfil linear no tempo que zera posição e velocidade no fim: a = 6·Δx/T² − 4·v/T.
		const double T2 = TimeToGo * TimeToGo;
		const double VerticalCommand = 6.0 * (Input.TargetRadius - Radius) / T2 - 4.0 * VerticalSpeed / TimeToGo;
		const double CrossCommand = -6.0 * CrossOffset / T2 - 4.0 * CrossSpeed / TimeToGo;

		// O empuxo precisa cobrir a gravidade menos o efeito centrífugo da velocidade horizontal.
		const double VerticalThrustNeeded = VerticalCommand + Input.Mu / (Radius * Radius)
			- (HorizontalSpeed * HorizontalSpeed + CrossSpeed * CrossSpeed) / Radius;

		const double VerticalFraction = std::clamp(VerticalThrustNeeded / Input.ThrustAcceleration,
			-GuidanceMaxVerticalFraction, GuidanceMaxVerticalFraction);
		const double CrossFraction = std::clamp(CrossCommand / Input.ThrustAcceleration,
			-GuidanceMaxCrossFraction, GuidanceMaxCrossFraction);
		const double DownrangeFraction = std::sqrt(std::max(0.0,
			1.0 - VerticalFraction * VerticalFraction - CrossFraction * CrossFraction));

		FClosedLoopCommand Out;
		Out.Direction = Up * VerticalFraction + Downrange * DownrangeFraction + Input.PlaneNormal * CrossFraction;
		Out.TimeToGo = TimeToGo;
		return Out;
	}
}
