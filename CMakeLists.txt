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
set(CMAKE_PROJECT_NAME vehical_rak_edge_ai)

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
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # ---- 本仓应用层（L3 解算 + 内容层；见 docs/architecture.md §3）----
    Core/Src/app_build.c
    Core/Src/app_oled_render.c
    Core/Src/app_serial.c
    Core/Src/app_bt.c
    Core/Src/app_tft.c
    Core/Src/app_ui.c
    Core/Src/app_ui_pages.c
    Core/Src/app_hooks.c

    # ---- platform/ 共用层（L1 驱动；submodule，两个 MCU 仓共用同一份源码）----
    platform/Src/app_oled.c
)

# Add include paths
# 顺序要紧：Core/Inc 放前面，让本仓的 board_c30d.h 覆盖 platform/ 的默认值
# （docs/architecture.md §4）。
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Inc
    platform/Inc
    Middlewares/Third_Party/LVGL
    Middlewares/Third_Party/LVGL/include
    Middlewares/Third_Party/LVGL/src
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # 让 LVGL 从 include 路径找 lv_conf.h（本工程放 Core/Inc/lv_conf.h）
    LV_CONF_INCLUDE_SIMPLE
)

# ---------------------------------------------------------------------------
# LVGL v9.6.0（vendored，见 Middlewares/Third_Party/LVGL/LICENCE.txt）
# 用 GLOB 收源文件：LVGL 的 .c 分布在 src/ 下 19 个子目录，逐条列举既长又
# 会在升级版本时漏掉新文件。新增/删除文件后需重新 configure（CMake 限制）。
# ---------------------------------------------------------------------------
file(GLOB_RECURSE LVGL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/LVGL/src/*.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${LVGL_SOURCES})

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

#
# ---------------------------------------------------------------------------
# 以下为手工添加。本文件只生成一次，CubeMX 重新生成不会覆盖；
# cmake/stm32cubemx/CMakeLists.txt 会被覆盖，自定义内容一律写在这里。
# ---------------------------------------------------------------------------
#
# 链接完成后生成烧录/存档产物 .hex 与 .bin（STM32CubeProgrammer 两者都收），
# 并打印一次体积占用，便于两端对比。
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMENT "Generating ${CMAKE_PROJECT_NAME}.hex / .bin"
)

