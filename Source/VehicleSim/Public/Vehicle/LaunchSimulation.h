#pragma once

#include <cstdint>
#include <vector>

#include "Orbital/Vec3.h"
#include "Vehicle/Guidance.h"
#include "Vehicle/VehicleDef.h"
#include "Vehicle/VehicleSimApi.h"

namespace Vehicle
{
	enum class EFlightPhase : uint8_t
	{
		VerticalAscent,
		Pitchover,
		GravityTurn,
		StagingCoast,
		ClosedLoop,
		Finished,
	};

	enum class EFlightEventType : uint8_t
	{
		Liftoff,
		PitchoverStart,
		MaxQ,
		FairingJettison,
		StageCutoff,
		StageSeparation,
		StageIgnition,
		EngineFailure,
		StructuralFailure,
		OrbitInsertion,
		Impact,
	};

	enum class EFlightOutcome : uint8_t
	{
		InFlight,
		OrbitAchieved,
		SuborbitalCutoff,    // cortou com a energia alvo, mas o perigeu ficou dentro da atmosfera
		PropellantExhausted, // o último estágio acabou antes da órbita
		StructuralFailure,   // pressão dinâmica acima do limite da estrutura
		Impact,
	};

	struct FFlightEvent
	{
		double Time = 0.0;
		EFlightEventType Type = EFlightEventType::Liftoff;
		int StageIndex = 0;
		double Value = 0.0; // MaxQ e StructuralFailure: pressão dinâmica (Pa); EngineFailure: motores perdidos
	};

	/** Uma falha agendada: a partir de Time, o estágio perde EnginesLost motores. */
	struct FEngineFailure
	{
		double Time = 0.0;
		int StageIndex = 0;
		int EnginesLost = 1;
	};

	struct FTelemetry
	{
		double Time = 0.0;
		EFlightPhase Phase = EFlightPhase::VerticalAscent;
		int StageIndex = 0;
		int ActiveEngines = 0;
		double Altitude = 0.0;           // m
		double InertialSpeed = 0.0;      // m/s
		double Airspeed = 0.0;           // m/s, relativa à atmosfera que gira com a Terra
		double Mach = 0.0;
		double DynamicPressure = 0.0;    // Pa
		double ThrustAcceleration = 0.0; // m/s²
		double Throttle = 0.0;           // 0..1
		double PitchAngle = 0.0;         // rad acima do horizonte local
		double Mass = 0.0;               // kg
		double DeltaVRemaining = 0.0;    // m/s, no vácuo
		double ApoapsisAltitude = 0.0;   // m
		double PeriapsisAltitude = 0.0;  // m
		double Inclination = 0.0;        // rad
	};

	/**
	 * Voo de um lançador da plataforma até o corte, em passos fixos. Acelerar o tempo só executa mais
	 * passos por quadro: o resultado é o mesmo em qualquer velocidade de tempo.
	 */
	class VEHICLESIM_API FLaunchSimulation
	{
	public:
		static constexpr double FixedTimeStep = 0.05; // s

		FLaunchSimulation(FVehicleDef InDefinition, FLaunchSite InSite, FAscentProfile InProfile);

		void ScheduleEngineFailure(const FEngineFailure& Failure);

		/** Avança um intervalo de tempo real (já multiplicado pelo time warp). Retorna os passos executados. */
		int Advance(double DeltaSeconds);
		void Step();
		void RunToCompletion(double MaxDuration = 3600.0);

		EFlightOutcome GetOutcome() const { return Outcome; }
		EFlightPhase GetPhase() const { return Phase; }
		double GetTime() const { return Time; }
		int64_t GetStepCount() const { return StepCount; }
		const Orbital::FStateVector& GetState() const { return State; }
		const FLaunchPlane& GetLaunchPlane() const { return Plane; }
		const std::vector<FFlightEvent>& GetEvents() const { return Events; }
		FTelemetry GetTelemetry() const;

	private:
		double CurrentMass() const;
		double ReferenceArea() const;
		double DeltaVRemaining() const;
		double AirFlightPathAngle() const;
		Orbital::FVec3 LaunchHeadingDirection() const;
		Orbital::FVec3 ComputeThrustDirection(double ThrustAcceleration);
		void ApplyDueEngineFailures();
		void UpdateFlightState();
		void HandleStageBurnout();
		void CutOffAtTargetEnergy();
		void AddEvent(EFlightEventType Type, int EventStage, double Value = 0.0);

		FVehicleDef Definition;
		FLaunchSite Site;
		FAscentProfile Profile;
		FLaunchPlane Plane;
		double KickAngle = 0.0;
		double TargetRadius = 0.0;
		double LaunchHeading = 0.0; // rad a partir do norte, na Terra que gira

		Orbital::FStateVector State;
		double Time = 0.0;
		int64_t StepCount = 0;
		double Accumulator = 0.0;

		EFlightPhase Phase = EFlightPhase::VerticalAscent;
		EFlightOutcome Outcome = EFlightOutcome::InFlight;
		int StageIndex = 0;
		std::vector<double> StagePropellant;
		std::vector<int> StageEngines;
		bool bFairingAttached = false;
		bool bLiftedOff = false;
		bool bClosedLoopEngaged = false;
		double StagingTimer = 0.0;

		Orbital::FVec3 LastDirection;
		double LastThrottle = 0.0;
		double LastThrustAcceleration = 0.0;

		double PeakDynamicPressure = 0.0;
		double PeakDynamicPressureTime = 0.0;
		bool bMaxQReported = false;

		std::vector<FEngineFailure> PendingFailures;
		std::vector<FFlightEvent> Events;
	};
}
