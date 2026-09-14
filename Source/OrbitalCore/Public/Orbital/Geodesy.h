#pragma once

#include "Orbital/OrbitalCoreApi.h"
#include "Orbital/Vec3.h"

// Geodésia sobre o elipsoide WGS84, o mesmo modelo de Terra do GPS e do Cesium.
//
// As coordenadas cartesianas têm Z no eixo de rotação. Latitude e altura não mudam com uma rotação em
// torno de Z, então as funções valem tanto para posições fixas na Terra quanto para posições inerciais;
// só a longitude passa a ser inercial nesse caso.
namespace Orbital
{
	struct FGeodeticCoordinates
	{
		double Latitude = 0.0;  // rad, geodésica
		double Longitude = 0.0; // rad
		double Height = 0.0;    // m acima do elipsoide
	};

	ORBITALCORE_API FVec3 GeodeticToCartesian(const FGeodeticCoordinates& Coordinates);

	/** Conversão inversa exata e sem iteração (Heikkinen, 1982). */
	ORBITALCORE_API FGeodeticCoordinates CartesianToGeodetic(const FVec3& Position);

	ORBITALCORE_API double GeodeticHeight(const FVec3& Position);

	/** Vertical local: a normal ao elipsoide no ponto abaixo da posição, apontando para fora. */
	ORBITALCORE_API FVec3 GeodeticUp(const FVec3& Position);
}
