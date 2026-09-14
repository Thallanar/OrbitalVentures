#include "Vehicle/Aerodynamics.h"

#include <iterator>

namespace Vehicle
{
	namespace
	{
		struct FDragCurvePoint
		{
			double Mach;
			double Coefficient;
		};

		// Curva típica de lançadores de corpo cilíndrico com ogiva: sobe na região transônica e cai no supersônico.
		constexpr FDragCurvePoint DragCurve[] = {
			{0.0, 0.30},
			{0.6, 0.30},
			{0.9, 0.40},
			{1.1, 0.55},
			{1.3, 0.52},
			{2.0, 0.40},
			{3.0, 0.32},
			{5.0, 0.27},
			{10.0, 0.25},
		};
	}

	double DragCoefficient(double Mach)
	{
		if (Mach <= DragCurve[0].Mach)
		{
			return DragCurve[0].Coefficient;
		}
		for (auto It = std::next(std::begin(DragCurve)); It != std::end(DragCurve); ++It)
		{
			if (Mach <= It->Mach)
			{
				const FDragCurvePoint& Low = *std::prev(It);
				const double Alpha = (Mach - Low.Mach) / (It->Mach - Low.Mach);
				return Low.Coefficient + Alpha * (It->Coefficient - Low.Coefficient);
			}
		}
		return std::prev(std::end(DragCurve))->Coefficient;
	}
}
