#pragma once

#include "Orbital/Vec3.h"

namespace Orbital
{
	/**
	 * Runge-Kutta de 4ª ordem. Accel(Time, State) -> FVec3 pode depender da velocidade
	 * (arrasto, empuxo), por isso é o integrador do voo propulsado e atmosférico.
	 */
	template <typename AccelFunc>
	FStateVector StepRK4(const FStateVector& State, double Time, double Dt, AccelFunc&& Accel)
	{
		const double HalfDt = 0.5 * Dt;

		const FVec3 A1 = Accel(Time, State);
		const FStateVector S2{State.Position + HalfDt * State.Velocity, State.Velocity + HalfDt * A1};
		const FVec3 A2 = Accel(Time + HalfDt, S2);
		const FStateVector S3{State.Position + HalfDt * S2.Velocity, State.Velocity + HalfDt * A2};
		const FVec3 A3 = Accel(Time + HalfDt, S3);
		const FStateVector S4{State.Position + Dt * S3.Velocity, State.Velocity + Dt * A3};
		const FVec3 A4 = Accel(Time + Dt, S4);

		const double SixthDt = Dt / 6.0;
		return {
			State.Position + SixthDt * (State.Velocity + 2.0 * S2.Velocity + 2.0 * S3.Velocity + S4.Velocity),
			State.Velocity + SixthDt * (A1 + 2.0 * A2 + 2.0 * A3 + A4)};
	}

	/**
	 * Velocity Verlet (simplético): o erro de energia fica limitado em vez de acumular.
	 * Só para forças conservativas que dependem apenas da posição: Accel(Position) -> FVec3.
	 */
	template <typename AccelFunc>
	FStateVector StepVelocityVerlet(const FStateVector& State, double Dt, AccelFunc&& Accel)
	{
		const FVec3 HalfStepVelocity = State.Velocity + (0.5 * Dt) * Accel(State.Position);
		const FVec3 NewPosition = State.Position + Dt * HalfStepVelocity;
		return {NewPosition, HalfStepVelocity + (0.5 * Dt) * Accel(NewPosition)};
	}
}
