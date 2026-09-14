#pragma once

#include <string>

#include "Orbital/Constants.h"
#include "Orbital/Vec3.h"
#include "Vehicle/VehicleSimApi.h"

namespace Vehicle
{
	/** Latitude e longitude geodésicas simplificadas (Terra esférica), em radianos. */
	struct FLaunchSite
	{
		std::string Name;
		double Latitude = 0.0;
		double Longitude = 0.0;
	};

	/** Os ajustes que o jogador faz antes do voo. */
	struct FAscentProfile
	{
		double TargetAltitude = 200e3;     // m, órbita circular alvo
		double TargetInclination = 0.0;    // rad; abaixo da latitude do sítio vira lançamento para leste
		double PitchoverAltitude = 1000.0; // m, onde começa o giro
		double Aggressiveness = 0.5;       // 0..1: quanto o foguete inclina no início do giro
		double MaxAcceleration = 4.0 * Orbital::Constants::StandardGravity; // m/s²; <= 0 desliga o limite
	};

	struct FLaunchPlane
	{
		Orbital::FVec3 Normal;           // normal do plano orbital alvo, no referencial inercial
		double Inclination = 0.0;        // rad, a inclinação que o plano realmente tem
		bool bInclinationReachable = true;
	};

	/**
	 * Plano orbital alvo a partir da posição inercial do sítio. Inclinações abaixo da latitude não são
	 * alcançáveis sem manobra de plano: o plano cai para o lançamento a leste e o resultado avisa.
	 */
	VEHICLESIM_API FLaunchPlane ComputeLaunchPlane(const Orbital::FVec3& SitePosition, double TargetInclination);

	/** Ângulo (rad) do chute inicial do giro gravitacional para uma agressividade 0..1. */
	VEHICLESIM_API double PitchoverKickAngle(double Aggressiveness);

	struct FClosedLoopInput
	{
		Orbital::FStateVector State;
		Orbital::FVec3 PlaneNormal;
		double TargetRadius = 0.0;       // m
		double Mu = 0.0;                 // m³/s²
		double ThrustAcceleration = 0.0; // m/s², com o estrangulamento atual
		double ExhaustVelocity = 0.0;    // m/s
	};

	struct FClosedLoopCommand
	{
		Orbital::FVec3 Direction; // unitário, inercial
		double TimeToGo = 0.0;    // s, estimativa até o corte
	};

	/**
	 * Guiagem em malha fechada da fase superior: recalcula a cada passo a aceleração vertical e lateral
	 * que leva o veículo à altitude alvo com velocidade vertical nula e ao plano alvo, com a velocidade
	 * horizontal crescendo até a circular. Perder empuxo muda a estimativa de tempo e a direção se
	 * ajusta sozinha — é isso que permite contornar a perda de um motor.
	 */
	VEHICLESIM_API FClosedLoopCommand ComputeClosedLoopCommand(const FClosedLoopInput& Input);
}
