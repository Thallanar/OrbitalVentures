#pragma once

// Harness mínimo de testes, sem dependências externas, para o OrbitalCore rodar fora do motor.

#include <cmath>
#include <cstdio>
#include <vector>

namespace TestHarness
{
	struct FTestCase
	{
		const char* Name;
		void (*Func)();
	};

	inline std::vector<FTestCase>& Registry()
	{
		static std::vector<FTestCase> Tests;
		return Tests;
	}

	inline int& FailureCount()
	{
		static int Count = 0;
		return Count;
	}

	struct FRegistrar
	{
		FRegistrar(const char* Name, void (*Func)()) { Registry().push_back({Name, Func}); }
	};

	inline void ReportFailure(const char* File, int Line, const char* Message)
	{
		std::printf("    FALHOU %s:%d: %s\n", File, Line, Message);
		++FailureCount();
	}
}

#define OV_TEST(Name) \
	static void Name(); \
	static const TestHarness::FRegistrar Name##Registrar(#Name, &Name); \
	static void Name()

#define OV_CHECK(Condition) \
	do { if (!(Condition)) { TestHarness::ReportFailure(__FILE__, __LINE__, #Condition); } } while (0)

#define OV_CHECK_NEAR(Actual, Expected, Tolerance) \
	do { \
		const double OvActual = (Actual); \
		const double OvExpected = (Expected); \
		const double OvTolerance = (Tolerance); \
		if (!(std::abs(OvActual - OvExpected) <= OvTolerance)) { \
			char OvBuffer[512]; \
			std::snprintf(OvBuffer, sizeof(OvBuffer), "%s = %.12g, esperado %.12g ± %.3g", \
				#Actual, OvActual, OvExpected, OvTolerance); \
			TestHarness::ReportFailure(__FILE__, __LINE__, OvBuffer); \
		} \
	} while (0)
