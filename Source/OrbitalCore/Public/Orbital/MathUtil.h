#pragma once

#include <cmath>

#include "Orbital/Constants.h"

namespace Orbital
{
	/** Normaliza um ângulo para [0, 2π). */
	inline double WrapTwoPi(double Angle)
	{
		const double Wrapped = std::fmod(Angle, Constants::TwoPi);
		return Wrapped < 0.0 ? Wrapped + Constants::TwoPi : Wrapped;
	}

	inline constexpr double DegToRad(double Degrees) { return Degrees * (Constants::Pi / 180.0); }
	inline constexpr double RadToDeg(double Radians) { return Radians * (180.0 / Constants::Pi); }
}
