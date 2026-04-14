cmake_minimum_required(VERSION 3.15)
project(Autotuning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(lib/FMI4cpp)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)

file(GLOB_RECURSE TUNING_SOURCES "tuning/*.cpp")
list(FILTER TUNING_SOURCES EXCLUDE REGEX "tuning/system/system_simulation\\.cpp$")
list(FILTER TUNING_SOURCES EXCLUDE REGEX "tuning/system/system_interface\\.cpp$")
add_library(tuning_lib ${TUNING_SOURCES})
target_include_directories(tuning_lib PUBLIC ${CMAKE_SOURCE_DIR})
target_link_libraries(tuning_lib PUBLIC statistics_lib)

file(GLOB_RECURSE STATISTICS_SOURCES "statistics/*.cpp")
add_library(statistics_lib ${STATISTICS_SOURCES})
target_include_directories(statistics_lib PUBLIC ${CMAKE_SOURCE_DIR})

add_library(simulation_lib
  simulation/ModelicaWrapper.cpp
  tuning/system/system_interface.cpp
  tuning/system/system_simulation.cpp
)
target_link_libraries(simulation_lib PUBLIC fmi4cpp statistics_lib tuning_lib)
target_include_directories(simulation_lib PUBLIC ${CMAKE_SOURCE_DIR})

add_executable(sim main.cpp autotuning_manager.cpp)
target_link_libraries(sim PRIVATE simulation_lib)
set_target_properties(sim PROPERTIES
    BUILD_RPATH "${CMAKE_BINARY_DIR}/lib/FMI4cpp/src"
    INSTALL_RPATH "${CMAKE_BINARY_DIR}/lib/FMI4cpp/src"
)

enable_testing()
include(GoogleTest)

function(create_test_target TARGET_NAME SOURCES)
    add_executable(${TARGET_NAME} ${SOURCES})
    target_link_libraries(${TARGET_NAME} PRIVATE simulation_lib gmock_main)
    set_target_properties(${TARGET_NAME} PROPERTIES
        BUILD_RPATH "${CMAKE_BINARY_DIR}/lib/FMI4cpp/src"
        INSTALL_RPATH "${CMAKE_BINARY_DIR}/lib/FMI4cpp/src"
    )
    gtest_discover_tests(${TARGET_NAME})
endfunction()

file(GLOB_RECURSE UNIT_TEST_SOURCES "test/unit/*.cpp")
if(UNIT_TEST_SOURCES)
    create_test_target(sim_unit_tests "${UNIT_TEST_SOURCES}")
endif()

# SundaresanTest.cpp tem main() (demo); nao ligar com gmock_main / gtest_discover_tests.
set(INTEGRATION_TEST_SOURCES test/integration/SimulationTest.cpp)
create_test_target(sim_integration_tests "${INTEGRATION_TEST_SOURCES}")

add_executable(sundaresan_demo test/integration/SundaresanTest.cpp)
target_link_libraries(sundaresan_demo PRIVATE simulation_lib)
set_target_properties(sundaresan_demo PROPERTIES
    BUILD_RPATH "${CMAKE_BINARY_DIR}/lib/FMI4cpp/src"
    INSTALL_RPATH "${CMAKE_BINARY_DIR}/lib/FMI4cpp/src"
)