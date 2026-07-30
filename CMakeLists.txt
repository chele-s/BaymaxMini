cmake_minimum_required(VERSION 3.16)
project(BaymaxMini VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(EXISTS "C:/boost")
    include_directories(SYSTEM "C:/boost")
else()
    find_package(Boost QUIET)
    if(Boost_FOUND)
        include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
    endif()
endif()

set(NATIVE_SOURCES
    src/cpp_core/serial/SerialLink.cpp
    src/cpp_core/analysis/AdvancedVitalsAnalyzer.cpp
    src/cpp_core/bindings/baymax_native.cpp
)

find_package(pybind11 CONFIG QUIET)
if(NOT pybind11_FOUND)
    execute_process(
        COMMAND python -m pybind11 --cmakedir
        OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(PYBIND11_CMAKE_DIR)
        list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
        find_package(pybind11 CONFIG REQUIRED)
    else()
        find_package(pybind11 REQUIRED)
    endif()
endif()

pybind11_add_module(baymax_native ${NATIVE_SOURCES})

target_include_directories(baymax_native PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp_core
)

if(MSVC)
    target_compile_options(baymax_native PRIVATE /W4 /O2 /permissive-)
else()
    target_compile_options(baymax_native PRIVATE -Wall -Wextra -O3 -pipe)
    if(UNIX AND NOT APPLE)
        target_link_libraries(baymax_native PRIVATE pthread)
    endif()
endif()
