cmake_minimum_required(VERSION 3.20)
project(rt_turbofan_afterburner_sim LANGUAGES CXX)

add_library(turbofan
    src/thermo.cpp
    src/nozzle.cpp
    src/performance_map.cpp
    src/map_dataset.cpp
    src/engine_config.cpp
    src/reference_conditions.cpp
    src/matching.cpp
    src/cycle.cpp
    src/transient.cpp
    src/controller.cpp
    src/control_profile.cpp
    src/actuator.cpp
)
target_include_directories(turbofan PUBLIC include)
target_compile_features(turbofan PUBLIC cxx_std_20)
set_target_properties(turbofan PROPERTIES CXX_EXTENSIONS OFF)

add_executable(turbofan-cli src/main.cpp src/public_comparison.cpp)
target_link_libraries(turbofan-cli PRIVATE turbofan)

include(CTest)
if(BUILD_TESTING)
    add_executable(thermo-test tests/thermo_test.cpp)
    target_link_libraries(thermo-test PRIVATE turbofan)
    add_test(NAME thermo COMMAND thermo-test)

    add_executable(nozzle-test tests/nozzle_test.cpp)
    target_link_libraries(nozzle-test PRIVATE turbofan)
    add_test(NAME nozzle COMMAND nozzle-test)

    add_executable(performance-map-test tests/performance_map_test.cpp)
    target_link_libraries(performance-map-test PRIVATE turbofan)
    add_test(NAME performance-map COMMAND performance-map-test)

    add_executable(map-dataset-test tests/map_dataset_test.cpp)
    target_link_libraries(map-dataset-test PRIVATE turbofan)
    add_test(NAME map-dataset COMMAND map-dataset-test)

    add_executable(engine-config-test tests/engine_config_test.cpp)
    target_link_libraries(engine-config-test PRIVATE turbofan)
    add_test(NAME engine-config COMMAND engine-config-test)

    add_executable(transient-test tests/transient_test.cpp)
    target_link_libraries(transient-test PRIVATE turbofan)
    add_test(NAME transient COMMAND transient-test)

    add_executable(controller-test tests/controller_test.cpp)
    target_link_libraries(controller-test PRIVATE turbofan)
    add_test(NAME controller COMMAND controller-test)

    add_executable(actuator-test tests/actuator_test.cpp)
    target_link_libraries(actuator-test PRIVATE turbofan)
    add_test(NAME actuator COMMAND actuator-test)

    add_executable(control-profile-test tests/control_profile_test.cpp)
    target_link_libraries(control-profile-test PRIVATE turbofan)
    add_test(NAME control-profile COMMAND control-profile-test)

    add_test(NAME controlled-cycle COMMAND turbofan-cli controlled)

    add_executable(reference-conditions-test tests/reference_conditions_test.cpp)
    target_link_libraries(reference-conditions-test PRIVATE turbofan)
    add_test(NAME reference-conditions COMMAND reference-conditions-test)

    add_test(NAME pw100-public-comparison COMMAND turbofan-cli compare-pw100)
    set_tests_properties(pw100-public-comparison PROPERTIES PASS_REGULAR_EXPRESSION "synthetic-similarity")

    add_executable(cycle-test tests/cycle_test.cpp)
    target_link_libraries(cycle-test PRIVATE turbofan)
    add_test(NAME cycle COMMAND cycle-test)
endif()
