cmake_minimum_required(VERSION 3.20)
project(PlanetGeneratorCore LANGUAGES CXX)

add_library(PlanetGeneratorCore STATIC Source/PlanetGenerator/Core/PlanetCore.cpp Source/PlanetGenerator/Core/PlanetLod.cpp Source/PlanetGenerator/Core/PlanetGeology.cpp
    Source/PlanetGenerator/Core/PlanetOcean.cpp
    Source/PlanetGenerator/Core/PlanetView.cpp
    Source/PlanetGenerator/Core/PlanetGround.cpp)
target_include_directories(PlanetGeneratorCore PUBLIC Source/PlanetGenerator/Core)
target_compile_features(PlanetGeneratorCore PUBLIC cxx_std_20)
add_executable(PlanetCoreTests Tests/Core/PlanetCoreTests.cpp)
target_link_libraries(PlanetCoreTests PRIVATE PlanetGeneratorCore)
add_executable(PlanetLodTests Tests/Core/PlanetLodTests.cpp)
target_link_libraries(PlanetLodTests PRIVATE PlanetGeneratorCore)
add_executable(PlanetGeologyTests Tests/Core/PlanetGeologyTests.cpp)
target_link_libraries(PlanetGeologyTests PRIVATE PlanetGeneratorCore)
if(MSVC)
    target_compile_options(PlanetGeneratorCore PRIVATE /W4 /WX /fp:precise)
    target_compile_options(PlanetCoreTests PRIVATE /W4 /WX /fp:precise)
    target_compile_options(PlanetLodTests PRIVATE /W4 /WX /fp:precise)
    target_compile_options(PlanetGeologyTests PRIVATE /W4 /WX /fp:precise)
else()
    target_compile_options(PlanetGeneratorCore PRIVATE -Wall -Wextra -Werror -fno-fast-math)
    target_compile_options(PlanetCoreTests PRIVATE -Wall -Wextra -Werror -fno-fast-math)
    target_compile_options(PlanetLodTests PRIVATE -Wall -Wextra -Werror -fno-fast-math)
    target_compile_options(PlanetGeologyTests PRIVATE -Wall -Wextra -Werror -fno-fast-math)
endif()
enable_testing()
add_test(NAME PlanetCoreContracts COMMAND PlanetCoreTests)

add_test(NAME PlanetLodContracts COMMAND PlanetLodTests)

add_test(NAME PlanetGeologyContracts COMMAND PlanetGeologyTests)
