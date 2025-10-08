cmake_minimum_required(VERSION 3.22)

# ------------------------------------------------------------
# Common configuration
# ------------------------------------------------------------
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME STM_TESTING)
project(${CMAKE_PROJECT_NAME} C ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

add_executable(${CMAKE_PROJECT_NAME})
add_subdirectory(cmake/stm32cubemx)

# ------------------------------------------------------------
# Linker script (dual setup)
# ------------------------------------------------------------
# 👉 For VS Code builds:
#     It will use the default STM32G431XX_FLASH.ld file in your STM32 folder.
# 👉 For App builds:
#     The Python tool passes the absolute path using -DLINKER_SCRIPT_PATH=...
# ------------------------------------------------------------
if(DEFINED ENV{LINKER_SCRIPT_PATH})
    message("Using linker script from environment: $ENV{LINKER_SCRIPT_PATH}")
    set(LINKER_SCRIPT "$ENV{LINKER_SCRIPT_PATH}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431XX_FLASH.ld")
    message("Using local linker script: ${CMAKE_CURRENT_SOURCE_DIR}/STM32G431XX_FLASH.ld")
    set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32G431XX_FLASH.ld")
else()
    message(FATAL_ERROR "❌ Linker script not found!")
endif()

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${LINKER_SCRIPT}
    --specs=nano.specs
    # --specs=nosys.specs
)

# ------------------------------------------------------------
# Includes, defines, and libraries
# ------------------------------------------------------------
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE)
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m   # ✅ math library (sinf, llround, etc.)
)

# Include any extra sources listed separately
include(extra_sources.cmake)
