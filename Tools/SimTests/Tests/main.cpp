#include <chrono>
#include <cstdio>

#include "TestHarness.h"

int main()
{
	int FailedTests = 0;
	for (const TestHarness::FTestCase& Test : TestHarness::Registry())
	{
		const int FailuresBefore = TestHarness::FailureCount();
		const auto Start = std::chrono::steady_clock::now();
		Test.Func();
		const double Ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - Start).count();
		const bool bPassed = TestHarness::FailureCount() == FailuresBefore;
		FailedTests += bPassed ? 0 : 1;
		std::printf("[%s] %s (%.1f ms)\n", bPassed ? "  OK  " : "FALHOU", Test.Name, Ms);
	}

	std::printf("\n%zu testes, %d falharam\n", TestHarness::Registry().size(), FailedTests);
	return FailedTests == 0 ? 0 : 1;
}
