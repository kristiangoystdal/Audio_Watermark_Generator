cmake_minimum_required(VERSION 3.22)

#
# Base CMakeLists for STM32 project
# User can extend freely
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
set(CMAKE_PROJECT_NAME Audio_Watermark_Generator)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431XX_FLASH.ld")

# Linker options
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${LINKER_SCRIPT}
    --specs=nano.specs
    --specs=nosys.specs
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths here
)

# Compile definitions
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined macros here
)

# Remove wrong libob.a dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Libraries to link
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m   # <-- math library (resolves sinf, llround, etc.)
)

# Add user sources if needed
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Include any extra sources listed separately
include(extra_sources.cmake)
