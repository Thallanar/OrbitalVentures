#include "Vehicle/VehicleDef.h"

#include <algorithm>

#include "Orbital/Constants.h"

namespace Vehicle
{
	namespace
	{
		constexpr double EngineReferenceSeaLevelPressure = 101325.0; // Pa
	}

	double MassFlowRate(const FEngineDef& Engine)
	{
		return Engine.VacuumThrust / (Engine.VacuumIsp * Orbital::Constants::StandardGravity);
	}

	double ThrustAtPressure(const FEngineDef& Engine, double AmbientPressure)
	{
		// A diferença entre o empuxo no vácuo e ao nível do mar define a área efetiva de saída do bocal.
		const double SeaLevelThrust = MassFlowRate(Engine) * Engine.SeaLevelIsp * Orbital::Constants::StandardGravity;
		const double ExitArea = (Engine.VacuumThrust - SeaLevelThrust) / EngineReferenceSeaLevelPressure;
		return std::max(0.0, Engine.VacuumThrust - ExitArea * AmbientPressure);
	}
}
