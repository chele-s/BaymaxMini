cmake_minimum_required(VERSION 3.16)
project(BaymaxMini
    VERSION     1.0.0
    LANGUAGES   CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(DRIVER_SOURCES
    src/cpp_core/drivers/I2C_Bus.cpp
    src/cpp_core/drivers/PCA9685.cpp
    src/cpp_core/drivers/VL53L1X.cpp
    src/cpp_core/drivers/MAX30102.cpp
    src/cpp_core/drivers/MLX90614.cpp
    src/cpp_core/drivers/INA219.cpp
)

set(MODULE_SOURCES
    src/cpp_core/modules/FaceController.cpp
    src/cpp_core/modules/VitalsMonitor.cpp
    src/cpp_core/modules/PowerSystem.cpp
)

set(MAIN_SOURCE
    src/cpp_core/main.cpp
)

add_executable(baymax_core
    ${MAIN_SOURCE}
    ${DRIVER_SOURCES}
    ${MODULE_SOURCES}
)

target_include_directories(baymax_core PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp_core
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)
find_package(nlohmann_json QUIET)

target_include_directories(baymax_core PRIVATE ${ZMQ_INCLUDE_DIRS})
target_link_libraries(baymax_core PRIVATE ${ZMQ_LIBRARIES})

if(nlohmann_json_FOUND)
    target_link_libraries(baymax_core PRIVATE nlohmann_json::nlohmann_json)
else()
    find_path(NLOHMANN_JSON_INCLUDE nlohmann/json.hpp
        PATHS /usr/include /usr/local/include
    )
    if(NLOHMANN_JSON_INCLUDE)
        target_include_directories(baymax_core PRIVATE ${NLOHMANN_JSON_INCLUDE})
    else()
        message(FATAL_ERROR "nlohmann/json.hpp not found. Install with: sudo apt install nlohmann-json3-dev")
    endif()
endif()

target_link_libraries(baymax_core PRIVATE pthread)

target_compile_options(baymax_core PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wno-unused-parameter
    -pipe
)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(baymax_core PRIVATE
        -O3
        -march=native
        -mtune=native
        -flto
        -ffast-math
        -fomit-frame-pointer
    )
    target_link_options(baymax_core PRIVATE -flto -s)
endif()

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(baymax_core PRIVATE
        -O0
        -g3
        -fsanitize=address,undefined
    )
    target_link_options(baymax_core PRIVATE -fsanitize=address,undefined)
endif()

add_library(baymax_drivers STATIC ${DRIVER_SOURCES})
target_include_directories(baymax_drivers PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp_core
)
target_compile_options(baymax_drivers PRIVATE -Wall -Wextra -O2)

add_library(baymax_modules STATIC ${MODULE_SOURCES})
target_include_directories(baymax_modules PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cpp_core
)
target_link_libraries(baymax_modules PUBLIC baymax_drivers)
target_compile_options(baymax_modules PRIVATE -Wall -Wextra -O2)

install(TARGETS baymax_core RUNTIME DESTINATION bin)

message(STATUS "BaymaxMini ${PROJECT_VERSION} - Build: ${CMAKE_BUILD_TYPE}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
