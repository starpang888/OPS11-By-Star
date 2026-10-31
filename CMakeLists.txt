cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME LED)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
set(FUSION_SOURCES
    modules/fusion/vendor/FusionAhrs.c
    modules/fusion/vendor/FusionBias.c
)
# Upstream forces many inline functions. Optimise the library in Debug too, to
# bound stack use and leave time for SPI acquisition within the 2 ms period.
set_source_files_properties(${FUSION_SOURCES} PROPERTIES COMPILE_OPTIONS "-O2")
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    bsp/bsp_g354.c
    bsp/bsp_mt6826s.c
    bsp/bsp_time.c
    modules/g354/g354.c
    modules/mt6826s/mt6826s.c
    modules/encoder_motion/encoder_motion.c
    modules/odometry/odometry.c
    modules/attitude/attitude.c
    ${FUSION_SOURCES}
    application/imu_app.c
    application/encoder_app.c
    application/odometry_app.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    bsp
    modules/g354
    modules/mt6826s
    modules/encoder_motion
    modules/odometry
    modules/attitude
    modules/fusion/vendor
    application
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    FUSION_USE_NORMAL_SQRT
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
