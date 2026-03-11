# extra_sources.cmake

# Collect all user .c files from Core/Src
file(GLOB USER_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/ds3231.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/ism.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/cc1101.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/radio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/frequency_config.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/led_feedback.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/relay.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/battery.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/reed_solomon.c
)

# Collect all user include directories
set(USER_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
)

# Add them to the main target
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${USER_SOURCES})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ${USER_INCLUDES})
