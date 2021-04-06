cmake_minimum_required(VERSION 3.10)
project(adaptiveShielding)

set(CMAKE_CXX_STANDARD 14)

find_package(Boost 1.65 COMPONENTS program_options system filesystem unit_test_framework REQUIRED)

include_directories(${Boost_INCLUDE_DIR})

SET(GCC_COVERAGE_COMPILE_FLAGS "-std=c++11 -pthread -lstdc++fs -g")
SET(GCC_COVERAGE_LINK_FLAGS    "-lstdc++fs -lboost_fiber -lboost_context")

add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

include_directories(include)

add_executable(adaptiveShielding
        src/main.cpp
        src/Shield.cpp
        src/ShieldConfig.cpp
        src/ShieldModelGenerator.cpp
        src/TrafficLight.cpp
        src/Strategy.cpp
        src/SUMOConnector.cpp
        src/STORMConnector.cpp
        src/Simulation.cpp
        src/Util.cpp
        src/LaneMapper.cpp
        src/PhaseMapper.cpp
        src/Controller.cpp
        src/Environment.cpp
        src/LaneTree.cpp
        src/TrafficIncidentManager.cpp
        lib/TraCIAPI.cpp
        lib/storage.cpp
        lib/socket.cpp)

target_link_libraries(adaptiveShielding ${Boost_LIBRARIES})
target_link_libraries(adaptiveShielding stdc++fs)