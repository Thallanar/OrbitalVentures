#pragma once

// Veículo e sítio de referência usados pelos testes e pelo LaunchSim. Números inspirados em um lançador
// real de dois estágios de querosene e oxigênio líquido, com nomes fictícios.

#include "Orbital/MathUtil.h"
#include "Vehicle/Guidance.h"
#include "Vehicle/VehicleDef.h"

namespace ReferenceVehicles
{
	inline Vehicle::FEngineDef MakeBoosterEngine()
	{
		return {.Name = "Harrier-1", .VacuumThrust = 914e3, .VacuumIsp = 311.0, .SeaLevelIsp = 282.0, .MinThrottle = 0.4};
	}

	inline Vehicle::FEngineDef MakeVacuumEngine()
	{
		return {.Name = "Harrier-1 Vac", .VacuumThrust = 981e3, .VacuumIsp = 348.0, .SeaLevelIsp = 150.0, .MinThrottle = 0.4};
	}

	inline Vehicle::FVehicleDef MakeKestrel9(double PayloadMass)
	{
		Vehicle::FVehicleDef Vehicle;
		Vehicle.Name = "Kestrel 9";
		Vehicle.Stages.push_back({.Name = "Kestrel 9 - estagio 1", .DryMass = 25600.0, .PropellantMass = 395700.0,
			.EngineCount = 9, .Engine = MakeBoosterEngine(), .Diameter = 3.66});
		Vehicle.Stages.push_back({.Name = "Kestrel 9 - estagio 2", .DryMass = 3900.0, .PropellantMass = 92670.0,
			.EngineCount = 1, .Engine = MakeVacuumEngine(), .Diameter = 3.66});
		Vehicle.PayloadMass = PayloadMass;
		Vehicle.FairingMass = 1900.0;
		Vehicle.FairingJettisonAltitude = 110e3;
		Vehicle.MaxDynamicPressure = 60e3;
		return Vehicle;
	}

	inline Vehicle::FLaunchSite MakeCapeCanaveral()
	{
		return {.Name = "Cabo Canaveral", .Latitude = Orbital::DegToRad(28.5623), .Longitude = Orbital::DegToRad(-80.5774)};
	}
}
