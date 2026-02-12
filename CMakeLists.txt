cmake_minimum_required(VERSION 3.15)
project(Autotuning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(lib/FMI4cpp)

include(FetchContent)

# Fetch GoogleTest
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
# Make googletest available
FetchContent_MakeAvailable(googletest)

# Collect statistics sources and include them in the simulation library
file(GLOB_RECURSE STATISTICS_SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "statistics/*.cpp")
if(STATISTICS_SOURCES)
    message(STATUS "Including statistics sources in simulation_lib: ${STATISTICS_SOURCES}")
endif()

add_library(simulation_lib
    simulation/ModelicaWrapper.cpp
    ${STATISTICS_SOURCES}
)

target_include_directories(simulation_lib
    PUBLIC ${CMAKE_SOURCE_DIR}
)

target_link_libraries(simulation_lib
    PUBLIC fmi4cpp
)

add_executable(sim
    main.cpp
)
target_link_libraries(sim PRIVATE simulation_lib)

# Collect unit and integration test sources (if any)
file(GLOB_RECURSE UNIT_TEST_SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "test/unit/*.cpp" "test/unit/**/*.cpp")
file(GLOB_RECURSE INTEGRATION_TEST_SOURCES RELATIVE ${CMAKE_SOURCE_DIR} "test/integration/*.cpp" "test/integration/**/*.cpp")

enable_testing()
include(GoogleTest)

if(UNIT_TEST_SOURCES)
    message(STATUS "Found unit test sources: ${UNIT_TEST_SOURCES}")
    add_executable(sim_unit_tests ${UNIT_TEST_SOURCES})
    target_include_directories(sim_unit_tests PRIVATE ${CMAKE_SOURCE_DIR})
    # Link with gmock_main so tests using Google Mock (<gmock/gmock.h>) build and run
    target_link_libraries(sim_unit_tests PRIVATE simulation_lib gmock_main)
    gtest_discover_tests(sim_unit_tests)
else()
    message(WARNING "No unit test sources found in test/unit. sim_unit_tests will not be created.")
endif()

if(INTEGRATION_TEST_SOURCES)
    message(STATUS "Found integration test sources: ${INTEGRATION_TEST_SOURCES}")
    add_executable(sim_integration_tests ${INTEGRATION_TEST_SOURCES})
    target_include_directories(sim_integration_tests PRIVATE ${CMAKE_SOURCE_DIR})
    # Use gmock_main as well to provide gmock headers if needed by integration tests
    target_link_libraries(sim_integration_tests PRIVATE simulation_lib gmock_main)
    gtest_discover_tests(sim_integration_tests)
else()
    message(WARNING "No integration test sources found in test/integration. sim_integration_tests will not be created.")
endif()
