cmake_minimum_required(VERSION 3.20)
project(LineSim C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 17)

if(NOT MSVC)
  message(FATAL_ERROR "LineSim currently builds with MSVC (Windows Fibers, fixed-address flash mapping).")
endif()

set(FW_DIR      ${CMAKE_SOURCE_DIR}/firmware)
set(MUJOCO_DIR  ${CMAKE_SOURCE_DIR}/third_party/mujoco)
set(SHIM_DIR    ${CMAKE_SOURCE_DIR}/sim/fw_shim)

if(NOT EXISTS ${FW_DIR}/Main/Src/drive.c)
  message(FATAL_ERROR "firmware submodule missing: run `git submodule update --init`")
endif()

# ---------------------------------------------------------------------------
# Firmware (compiled unmodified as C, on top of the HAL shim)
# ---------------------------------------------------------------------------
set(FW_SOURCES
  Main/Src/foc.c
  Main/Src/sensor.c
  Main/Src/motor.c
  Main/Src/drive.c
  Main/Src/menu.c
  Main/Src/sd_ui.c
  Main/Src/user_init.c
  Drivers/BSP/Button/button.c
  Drivers/BSP/FUET-8540/buzzer.c
  Drivers/BSP/LSM6DS3TR-C/lsm6ds3tr-c.c
  Drivers/BSP/SDcard/SDcard.c
)

# MSVC's C compiler rejects a few GCC-tolerated constructs (e.g. `default:` directly
# followed by `}`). Copy the sources into the build tree with those spots fixed;
# the firmware repo itself is never touched. Re-runs automatically when a source changes.
set(FW_PATCHED_DIR ${CMAKE_BINARY_DIR}/fw_patched)
set(FW_PATCHED_SOURCES "")
foreach(src ${FW_SOURCES})
  set(in  ${FW_DIR}/${src})
  set(out ${FW_PATCHED_DIR}/${src})
  file(READ ${in} text)
  string(REGEX REPLACE "default:([ \t\r\n]*)}" "default: ;\\1}" text "${text}")
  # #line keeps compiler messages and debugger breakpoints on the original firmware file
  # (the fix above never changes the line count).
  file(WRITE ${out}.tmp "#line 1 \"${in}\"\n${text}")
  configure_file(${out}.tmp ${out} COPYONLY)   # only touches ${out} when the content changed
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${in})
  list(APPEND FW_PATCHED_SOURCES ${out})
endforeach()

set(FW_INCLUDE_DIRS
  ${SHIM_DIR}/include
  ${FW_DIR}/Core/Inc
  ${FW_DIR}/Main/Inc
  ${FW_DIR}/Drivers/BSP/Button
  ${FW_DIR}/Drivers/BSP/FUET-8540
  ${FW_DIR}/Drivers/BSP/LSM6DS3TR-C
  ${FW_DIR}/Drivers/BSP/DRV8316C
  ${FW_DIR}/Drivers/BSP/MCT8316Z
  ${FW_DIR}/Drivers/BSP/MT6701
  ${FW_DIR}/Drivers/BSP/SDcard
  ${FW_DIR}/Drivers/BSP/w25qxx
)
set(FW_C_WARNINGS /wd4244 /wd4305 /wd4267 /wd4311 /wd4312 /wd4018 /wd4146 /wd4090 /wd4101 /wd4996 /wd4068 /wd4113)

# The firmware itself: every loop condition yields to the simulator (sim_fw_prelude.h).
add_library(firmware STATIC ${FW_PATCHED_SOURCES})
target_include_directories(firmware PUBLIC ${FW_INCLUDE_DIRS})
target_compile_definitions(firmware PUBLIC _CRT_SECURE_NO_WARNINGS SIM_FW_REAL_MAIN_H="${FW_DIR}/Core/Inc/main.h")
target_compile_options(firmware PRIVATE /utf-8 /W3 /FIsim_fw_prelude.h ${FW_C_WARNINGS})

# HAL / device stand-ins and the simulator bridge: plain C, no loop hook.
add_library(fw_shim STATIC
  ${SHIM_DIR}/src/hal_shim.c
  ${SHIM_DIR}/src/ff_shim.c
  ${SHIM_DIR}/src/devices_shim.c
  ${SHIM_DIR}/src/fw_bridge.c
)
target_include_directories(fw_shim PUBLIC ${FW_INCLUDE_DIRS})
target_compile_definitions(fw_shim PUBLIC _CRT_SECURE_NO_WARNINGS SIM_FW_REAL_MAIN_H="${FW_DIR}/Core/Inc/main.h")
target_compile_options(fw_shim PRIVATE /utf-8 /W3 /FIsim_compat.h ${FW_C_WARNINGS})

# ---------------------------------------------------------------------------
# MuJoCo (official prebuilt binaries)
# ---------------------------------------------------------------------------
add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
  IMPORTED_LOCATION ${MUJOCO_DIR}/bin/mujoco.dll
  IMPORTED_IMPLIB   ${MUJOCO_DIR}/lib/mujoco.lib
  INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_DIR}/include
)

# ---------------------------------------------------------------------------
# Simulator
# ---------------------------------------------------------------------------
option(LINESIM_BUILD_APP "Build the simulator executable" ON)
if(LINESIM_BUILD_APP)
add_executable(linesim
  sim/app/main.cpp
  sim/core/json.cpp
  sim/core/simulation.cpp
  sim/core/scenario.cpp
  sim/world/track.cpp
  sim/world/track_gen.cpp
  sim/world/robot.cpp
  sim/world/sensors.cpp
  sim/out/report.cpp
  sim/view/viewer.cpp
  sim/view/live_view.cpp
)
target_include_directories(linesim PRIVATE sim ${SHIM_DIR}/include)
target_link_libraries(linesim PRIVATE firmware fw_shim mujoco opengl32 gdi32 user32 winmm)
target_compile_options(linesim PRIVATE /utf-8 /W3 /wd4996)
target_compile_definitions(linesim PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
# The firmware stores function pointers and RAM addresses in 32-bit integers
# (menu.c "Last Used", HAL_FLASH_Program). Keep the whole process below 2 GB.
target_link_options(linesim PRIVATE /LARGEADDRESSAWARE:NO /DYNAMICBASE:NO /BASE:0x20000000 /STACK:8388608)

add_custom_command(TARGET linesim POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${MUJOCO_DIR}/bin/mujoco.dll $<TARGET_FILE_DIR:linesim>)
endif()
