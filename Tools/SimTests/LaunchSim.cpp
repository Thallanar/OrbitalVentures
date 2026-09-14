// Transmissão de um lançamento em linha de comando: telemetria periódica e eventos, como a sala de controle veria.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "Orbital/MathUtil.h"
#include "ReferenceVehicles.h"
#include "Vehicle/LaunchSimulation.h"

namespace
{
	const char* PhaseLabel(Vehicle::EFlightPhase Phase)
	{
		switch (Phase)
		{
		case Vehicle::EFlightPhase::VerticalAscent: return "vertical";
		case Vehicle::EFlightPhase::Pitchover: return "giro";
		case Vehicle::EFlightPhase::GravityTurn: return "gravidade";
		case Vehicle::EFlightPhase::StagingCoast: return "separacao";
		case Vehicle::EFlightPhase::ClosedLoop: return "malha fech.";
		case Vehicle::EFlightPhase::Finished: return "fim";
		}
		return "?";
	}

	void PrintEvent(const Vehicle::FFlightEvent& Event)
	{
		const int Minutes = static_cast<int>(Event.Time) / 60;
		const double Seconds = Event.Time - Minutes * 60;
		std::printf("  T+%02d:%04.1f  >> ", Minutes, Seconds);
		const int Stage = Event.StageIndex + 1;
		switch (Event.Type)
		{
		case Vehicle::EFlightEventType::Liftoff: std::printf("DECOLAGEM\n"); break;
		case Vehicle::EFlightEventType::PitchoverStart: std::printf("INICIO DO GIRO\n"); break;
		case Vehicle::EFlightEventType::MaxQ: std::printf("MAX-Q (%.1f kPa)\n", Event.Value / 1000.0); break;
		case Vehicle::EFlightEventType::FairingJettison: std::printf("SEPARACAO DA CARENAGEM\n"); break;
		case Vehicle::EFlightEventType::StageCutoff: std::printf("CORTE DO MOTOR - ESTAGIO %d\n", Stage); break;
		case Vehicle::EFlightEventType::StageSeparation: std::printf("SEPARACAO DO ESTAGIO %d\n", Stage); break;
		case Vehicle::EFlightEventType::StageIgnition: std::printf("IGNICAO - ESTAGIO %d\n", Stage); break;
		case Vehicle::EFlightEventType::EngineFailure: std::printf("FALHA: %.0f motor(es) perdido(s) no estagio %d\n", Event.Value, Stage); break;
		case Vehicle::EFlightEventType::StructuralFailure: std::printf("FALHA ESTRUTURAL (%.1f kPa)\n", Event.Value / 1000.0); break;
		case Vehicle::EFlightEventType::OrbitInsertion: std::printf("CORTE - INJECAO EM ORBITA\n"); break;
		case Vehicle::EFlightEventType::Impact: std::printf("IMPACTO\n"); break;
		}
	}

	void PrintRow(const Vehicle::FTelemetry& T)
	{
		const int Minutes = static_cast<int>(T.Time) / 60;
		const int Seconds = static_cast<int>(T.Time) % 60;
		std::printf("  T+%02d:%02d   %-11s %2d %8.1f %8.0f %7.1f %5.2f %6.1f %8.0f %9.1f %9.1f\n",
			Minutes, Seconds, PhaseLabel(T.Phase), T.StageIndex + 1, T.Altitude / 1000.0, T.InertialSpeed,
			T.DynamicPressure / 1000.0, T.ThrustAcceleration / Orbital::Constants::StandardGravity,
			Orbital::RadToDeg(T.PitchAngle), T.DeltaVRemaining, T.ApoapsisAltitude / 1000.0, T.PeriapsisAltitude / 1000.0);
	}

	void PrintUsage()
	{
		std::printf(
			"Uso: LaunchSim [opcoes]\n"
			"  --payload <t>          massa da carga (padrao 10)\n"
			"  --altitude <km>        orbita circular alvo (padrao 200)\n"
			"  --site <alcantara|cabo> sitio de lancamento (padrao alcantara)\n"
			"  --inclination <graus>  inclinacao alvo (padrao: latitude do sitio)\n"
			"  --pitchover <m>        altitude do inicio do giro (padrao 1000)\n"
			"  --aggressiveness <0-1> agressividade do giro (padrao 0.5)\n"
			"  --engine-out <T>:<estagio>:<motores>  falha agendada, estagio a partir de 1\n"
			"  --interval <s>         intervalo da telemetria (padrao 10)\n");
	}
}

int main(int Argc, char** Argv)
{
	double PayloadTonnes = 10.0;
	double IntervalSeconds = 10.0;
	Vehicle::FLaunchSite LaunchSite = ReferenceVehicles::MakeAlcantara();
	Vehicle::FAscentProfile Ascent;
	bool bInclinationGiven = false;
	std::vector<Vehicle::FEngineFailure> Failures;

	for (int Index = 1; Index < Argc; ++Index)
	{
		const char* Arg = Argv[Index];
		const bool bHasValue = Index + 1 < Argc;
		if (std::strcmp(Arg, "--help") == 0 || !bHasValue)
		{
			PrintUsage();
			return std::strcmp(Arg, "--help") == 0 ? 0 : 1;
		}
		const char* Value = Argv[++Index];
		if (std::strcmp(Arg, "--payload") == 0) { PayloadTonnes = std::atof(Value); }
		else if (std::strcmp(Arg, "--altitude") == 0) { Ascent.TargetAltitude = std::atof(Value) * 1000.0; }
		else if (std::strcmp(Arg, "--inclination") == 0) { Ascent.TargetInclination = Orbital::DegToRad(std::atof(Value)); bInclinationGiven = true; }
		else if (std::strcmp(Arg, "--site") == 0)
		{
			if (std::strcmp(Value, "alcantara") == 0) { LaunchSite = ReferenceVehicles::MakeAlcantara(); }
			else if (std::strcmp(Value, "cabo") == 0) { LaunchSite = ReferenceVehicles::MakeCapeCanaveral(); }
			else
			{
				PrintUsage();
				return 1;
			}
		}
		else if (std::strcmp(Arg, "--pitchover") == 0) { Ascent.PitchoverAltitude = std::atof(Value); }
		else if (std::strcmp(Arg, "--aggressiveness") == 0) { Ascent.Aggressiveness = std::atof(Value); }
		else if (std::strcmp(Arg, "--interval") == 0) { IntervalSeconds = std::atof(Value); }
		else if (std::strcmp(Arg, "--engine-out") == 0)
		{
			Vehicle::FEngineFailure Failure;
			if (std::sscanf(Value, "%lf:%d:%d", &Failure.Time, &Failure.StageIndex, &Failure.EnginesLost) != 3)
			{
				PrintUsage();
				return 1;
			}
			Failure.StageIndex -= 1;
			Failures.push_back(Failure);
		}
		else
		{
			PrintUsage();
			return 1;
		}
	}

	if (!bInclinationGiven)
	{
		// Sem inclinação pedida, lança para leste: a menor inclinação que o sítio alcança é a própria latitude.
		Ascent.TargetInclination = std::abs(LaunchSite.Latitude);
	}

	Vehicle::FLaunchSimulation Sim(ReferenceVehicles::MakeKestrel9(PayloadTonnes * 1000.0), LaunchSite, Ascent);
	for (const Vehicle::FEngineFailure& Failure : Failures)
	{
		Sim.ScheduleEngineFailure(Failure);
	}

	std::printf("\n  %s | carga %.1f t | alvo %.0f km, %.2f graus | sitio %s\n",
		"Kestrel 9", PayloadTonnes, Ascent.TargetAltitude / 1000.0, Orbital::RadToDeg(Sim.GetLaunchPlane().Inclination), LaunchSite.Name.c_str());
	if (!Sim.GetLaunchPlane().bInclinationReachable)
	{
		std::printf("  AVISO: inclinacao abaixo da latitude do sitio; lancando para leste.\n");
	}
	std::printf("\n  %-8s  %-11s %2s %8s %8s %7s %5s %6s %8s %9s %9s\n",
		"tempo", "fase", "E", "alt km", "vel m/s", "Q kPa", "acel", "pitch", "dv m/s", "apo km", "peri km");

	size_t PrintedEvents = 0;
	double NextRowTime = 0.0;
	while (Sim.GetOutcome() == Vehicle::EFlightOutcome::InFlight && Sim.GetTime() < 3600.0)
	{
		if (Sim.GetTime() >= NextRowTime - 1e-9)
		{
			PrintRow(Sim.GetTelemetry());
			NextRowTime += IntervalSeconds;
		}
		Sim.Step();
		const std::vector<Vehicle::FFlightEvent>& Events = Sim.GetEvents();
		for (; PrintedEvents < Events.size(); ++PrintedEvents)
		{
			PrintEvent(Events[PrintedEvents]);
		}
	}
	PrintRow(Sim.GetTelemetry());

	const Vehicle::FTelemetry Final = Sim.GetTelemetry();
	const char* OutcomeText = "em voo";
	switch (Sim.GetOutcome())
	{
	case Vehicle::EFlightOutcome::OrbitAchieved: OutcomeText = "ORBITA ALCANCADA"; break;
	case Vehicle::EFlightOutcome::SuborbitalCutoff: OutcomeText = "CORTE SUBORBITAL"; break;
	case Vehicle::EFlightOutcome::PropellantExhausted: OutcomeText = "PROPELENTE ESGOTADO ANTES DA ORBITA"; break;
	case Vehicle::EFlightOutcome::StructuralFailure: OutcomeText = "FALHA ESTRUTURAL"; break;
	case Vehicle::EFlightOutcome::Impact: OutcomeText = "IMPACTO"; break;
	case Vehicle::EFlightOutcome::InFlight: break;
	}
	std::printf("\n  Resultado: %s\n  Orbita: %.1f x %.1f km, inclinacao %.2f graus | dv restante %.0f m/s | tempo de voo %.0f s\n\n",
		OutcomeText, Final.ApoapsisAltitude / 1000.0, Final.PeriapsisAltitude / 1000.0,
		Orbital::RadToDeg(Final.Inclination), Final.DeltaVRemaining, Final.Time);
	return Sim.GetOutcome() == Vehicle::EFlightOutcome::OrbitAchieved ? 0 : 2;
}
