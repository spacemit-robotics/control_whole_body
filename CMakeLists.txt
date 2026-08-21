cmake_minimum_required(VERSION 3.16)
project(whole_body LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(MOTOR_LIBRARY NAMES motor
    PATHS ${CMAKE_INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_path(MOTOR_INCLUDE_DIR NAMES motor.h
    PATHS ${CMAKE_INSTALL_PREFIX}/include NO_DEFAULT_PATH)
find_library(IMU_LIBRARY NAMES imu
    PATHS ${CMAKE_INSTALL_PREFIX}/lib NO_DEFAULT_PATH)
find_path(IMU_INCLUDE_DIR NAMES imu.h
    PATHS ${CMAKE_INSTALL_PREFIX}/include NO_DEFAULT_PATH)

if(NOT MOTOR_LIBRARY OR NOT MOTOR_INCLUDE_DIR)
    message(FATAL_ERROR "whole_body requires the installed motor component")
endif()
if(NOT IMU_LIBRARY OR NOT IMU_INCLUDE_DIR)
    message(FATAL_ERROR "whole_body requires the installed imu component")
endif()

add_library(whole_body SHARED
    src/device_manager.cpp
    src/parallel_ankle.cpp
    src/whole_body.cpp
    src/whole_body_config.cpp
    src/whole_body_core.cpp
)

target_include_directories(whole_body
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${MOTOR_INCLUDE_DIR}
        ${IMU_INCLUDE_DIR}
)

target_link_libraries(whole_body
    PRIVATE
        yaml-cpp
        ${IMU_LIBRARY}
        -Wl,--whole-archive
        ${MOTOR_LIBRARY}
        -Wl,--no-whole-archive
        Threads::Threads
        dl
        rt
        m
)

set_target_properties(whole_body PROPERTIES
    BUILD_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH "$ORIGIN"
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(whole_body PRIVATE -Wall -Wextra -Werror)
endif()

install(TARGETS whole_body
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
install(FILES include/whole_body.h DESTINATION include)

option(BUILD_TESTS "Build whole-body offline tests" ON)
if(BUILD_TESTS)
    enable_testing()

    add_executable(test_whole_body_config tests/test_config.cpp)
    target_include_directories(test_whole_body_config BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${MOTOR_INCLUDE_DIR}
        ${IMU_INCLUDE_DIR}
    )
    target_compile_definitions(test_whole_body_config PRIVATE
        TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")
    target_link_libraries(test_whole_body_config PRIVATE whole_body)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_whole_body_config PRIVATE -UNDEBUG)
    endif()
    add_test(NAME whole_body_config COMMAND test_whole_body_config)

    add_executable(test_parallel_ankle tests/test_parallel_ankle.cpp)
    target_include_directories(test_parallel_ankle BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(test_parallel_ankle PRIVATE whole_body)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_parallel_ankle PRIVATE -UNDEBUG)
    endif()
    add_test(NAME parallel_ankle COMMAND test_parallel_ankle)

    add_executable(test_whole_body_core tests/test_core.cpp)
    target_include_directories(test_whole_body_core BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${MOTOR_INCLUDE_DIR}
        ${IMU_INCLUDE_DIR}
    )
    target_link_libraries(test_whole_body_core PRIVATE whole_body)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(test_whole_body_core PRIVATE -UNDEBUG)
    endif()
    add_test(NAME whole_body_core COMMAND test_whole_body_core)

    install(TARGETS
        test_whole_body_config
        test_parallel_ankle
        test_whole_body_core
        RUNTIME DESTINATION bin
    )
endif()
