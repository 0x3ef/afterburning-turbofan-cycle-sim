cmake_minimum_required(VERSION 3.20)
project(rt_turbofan_afterburner_sim LANGUAGES CXX)

add_library(turbofan
    src/thermo.cpp
    src/nozzle.cpp
    src/performance_map.cpp
    src/map_dataset.cpp
    src/engine_config.cpp
    src/matching.cpp
    src/cycle.cpp
)
target_include_directories(turbofan PUBLIC include)
target_compile_features(turbofan PUBLIC cxx_std_20)
set_target_properties(turbofan PROPERTIES CXX_EXTENSIONS OFF)

add_executable(turbofan-cli src/main.cpp)
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

    add_executable(cycle-test tests/cycle_test.cpp)
    target_link_libraries(cycle-test PRIVATE turbofan)
    add_test(NAME cycle COMMAND cycle-test)
endif()
