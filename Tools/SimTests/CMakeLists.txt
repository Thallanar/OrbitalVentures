# Build standalone do núcleo de simulação (OrbitalCore + VehicleSim), sem Unreal Engine.
#   cmake -S Tools/SimTests -B Tools/SimTests/build -DCMAKE_BUILD_TYPE=Release
#   cmake --build Tools/SimTests/build
#   ./Tools/SimTests/build/SimTests          # testes
#   ./Tools/SimTests/build/LaunchSim --help  # lançamento em linha de comando
cmake_minimum_required(VERSION 3.20)
project(SimTests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

# -Wshadow espelha o UBT, que trata shadowing como erro.
add_compile_options(-Wall -Wextra -Wshadow -Werror)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../Source)
# Só as pastas Private/Orbital e Private/Vehicle: os arquivos de módulo do UE dependem do motor.
file(GLOB SIM_CORE_SOURCES CONFIGURE_DEPENDS
	${SOURCE_DIR}/OrbitalCore/Private/Orbital/*.cpp
	${SOURCE_DIR}/VehicleSim/Private/Vehicle/*.cpp)

add_library(SimCore STATIC ${SIM_CORE_SOURCES})
target_include_directories(SimCore PUBLIC
	${SOURCE_DIR}/OrbitalCore/Public
	${SOURCE_DIR}/VehicleSim/Public)

file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/Tests/*.cpp)
add_executable(SimTests ${TEST_SOURCES})
target_include_directories(SimTests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(SimTests PRIVATE SimCore)

add_executable(LaunchSim LaunchSim.cpp)
target_link_libraries(LaunchSim PRIVATE SimCore)

enable_testing()
add_test(NAME SimTests COMMAND SimTests)
