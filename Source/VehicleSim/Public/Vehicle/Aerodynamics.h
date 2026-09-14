#pragma once

#include "Vehicle/VehicleSimApi.h"

namespace Vehicle
{
	/** Coeficiente de arrasto de um foguete esbelto em função do número de Mach, com o pico transônico. */
	VEHICLESIM_API double DragCoefficient(double Mach);
}
