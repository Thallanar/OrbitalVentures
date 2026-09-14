#pragma once

#include <string>
#include <vector>

#include "Vehicle/VehicleSimApi.h"

// Definições de veículo em unidades SI: newtons, segundos, quilogramas, metros.
namespace Vehicle
{
	struct FEngineDef
	{
		std::string Name;
		double VacuumThrust = 0.0; // N
		double VacuumIsp = 0.0;    // s
		double SeaLevelIsp = 0.0;  // s
		double MinThrottle = 1.0;  // fração do empuxo máximo; 1 = motor não estrangula
	};

	struct FStageDef
	{
		std::string Name;
		double DryMass = 0.0;        // kg
		double PropellantMass = 0.0; // kg
		int EngineCount = 1;
		FEngineDef Engine;
		double Diameter = 0.0;       // m
	};

	/** Estágios em ordem de queima: o índice 0 é o primeiro a acender. */
	struct FVehicleDef
	{
		std::string Name;
		std::vector<FStageDef> Stages;
		double PayloadMass = 0.0;             // kg
		double FairingMass = 0.0;             // kg; 0 = sem carenagem
		double FairingJettisonAltitude = 0.0; // m
		double MaxDynamicPressure = 0.0;      // Pa; acima disso a estrutura falha. 0 = sem limite
	};

	/** Vazão mássica com empuxo máximo. Independe da pressão externa. */
	VEHICLESIM_API double MassFlowRate(const FEngineDef& Engine);

	/** Empuxo máximo sob uma pressão externa (Pa): a pressão ambiente empurra contra a área de saída do bocal. */
	VEHICLESIM_API double ThrustAtPressure(const FEngineDef& Engine, double AmbientPressure);
}
