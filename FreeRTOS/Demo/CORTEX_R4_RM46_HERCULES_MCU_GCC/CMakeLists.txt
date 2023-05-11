# First round of a CMakeLists.txt for the Hercules RM46L852 MCU

set(CMAKE_CROSSCOMPILING "TRUE" CACHE STRING "Set Cross Compiling to true" FORCE)

# As many of us use Macs, leaving the stripping of MacOSX features.
set(CMAKE_OSX_DEPLOYMENT_TARGET "" CACHE STRING "Force unset of the deployment target for iOS" FORCE)
set(CMAKE_OSX_SYSROOT "" CACHE STRING "Force unset of the deployment target for iOS" FORCE)

# Need to set the compiler before declaring the project for the test build
set(CMAKE_C_COMPILER "arm-none-eabi-gcc")
set(CMAKE_ASM_COMPILER "arm-none-eabi-gcc")

# Set the system processor and name before declaring the project
# Needs to be set here otherwise it will fail the test compilation
set(CMAKE_SYSTEM_NAME "Generic" CACHE STRING "Target system is a generic ARM Processor.")
set(CMAKE_SYSTEM_PROCESSOR "armv7-r" CACHE STRING "Target system is an ARM7r processor")

## TODO: See if there are different CMAKE flags that can be set to do this instead of this
# Set the ASM flags, then add the extra ones for the CFlags
set(CMAKE_ASM_FLAGS "-mcpu=cortex-r4f -mfpu=vfpv3-d16 -Og -g -ggdb -Wall")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -specs=\"nosys.specs\" -specs=\"nano.specs\" -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_ASM_FLAGS} -marm -mfloat-abi=hard")

project(RM46_FreeRTOS C ASM)
cmake_minimum_required(VERSION 3.5)

# -> BUILD_DIR := build
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR} CACHE STRING "")

# For debug, no change detected ...
set(CMAKE_VERBOSE_MAKEFILE ON)

# Get the abosulte path to the Board Files
set(BOARD_FILES_DIR_REL "${CMAKE_CURRENT_SOURCE_DIR}/BoardFiles/")
get_filename_component(BOARD_FILES_DIR ${BOARD_FILES_DIR_REL} ABSOLUTE)

# Get the absolute path to the FREERTOS Directory
set(FREERTOS_DIR_REL "${CMAKE_CURRENT_SOURCE_DIR}/../../Source")
get_filename_component(FREERTOS_DIR ${FREERTOS_DIR_REL} ABSOLUTE)

# Get the absolute path to the Port Directory
set(PORT_DIR_REL "${CMAKE_CURRENT_SOURCE_DIR}/../../Source")
get_filename_component(PORT_DIR ${PORT_DIR_REL} ABSOLUTE)

# Get the absolute path to the Common Demo Directory
set(COMMON_DEMO_TASKS_DIR_REL "${CMAKE_CURRENT_SOURCE_DIR}/../Common")
get_filename_component(COMMON_DEMO_TASKS_DIR ${COMMON_DEMO_TASKS_DIR_REL} ABSOLUTE)

include_directories(
    ${DEMO_DIR}
    ${BOARD_FILES_DIR}/include
    ${PORT_DIR}/portable/GCC/ARM_CRx_No_GIC
    ${COMMON_DEMO_TASKS_DIR}/include
    ${COMMON_DEMO_TASKS_DIR}/MPU_Demo
    ${FREERTOS_DIR}/include
)

# Source files used for the FreeRTOS Demos
FILE(GLOB FREERTOS_DEMO_SOURCES
    ${BOARD_FILES_DIR}/source/*.c
    ${BOARD_FILES_DIR}/source/*.s  
    ${COMMON_DEMO_TASKS_DIR}/Minimal/AbortDelay.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/blocktim.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/countsem.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/death.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/dynamic.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/EventGroupsDemo.c 
    ${COMMON_DEMO_TASKS_DIR}/Minimal/GenQTest.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/IntSemTest.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/MessageBufferAMP.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/MessageBufferDemo.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/PollQ.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/QPeek.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/QueueOverwrite.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/QueueSet.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/QueueSetPolling.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/recmutex.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/semtest.c  
    ${COMMON_DEMO_TASKS_DIR}/Minimal/StaticAllocation.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/StreamBufferDemo.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/StreamBufferInterrupt.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/TaskNotify.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/TaskNotifyArray.c
    ${COMMON_DEMO_TASKS_DIR}/Minimal/TimerDemo.c
)

# FreeRTOS Kernel Files
FILE(GLOB FREERTOS_KERNEL_SOURCES
    ${FREERTOS_DIR}/croutine.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/timers.c
)

FILE(GLOB FREERTOS_PORT_SOURCES
    ${PORT_DIR}/portable/Common/mpu_wrappers.c
    ${PORT_DIR}/portable/GCC/ARM_CRx_No_GIC/portASM.S
    ${PORT_DIR}/portable/GCC/ARM_CRx_No_GIC/port.c
    # FREERTOS Heap
    ${PORT_DIR}/portable/MemMang/heap_4.c
)

# On Mac the C_LINK flags by default adds "-Wl,-search_paths_first -Wl,-headerpad_max_install_names" which
# Causes the executable that gets built to strip the symbols, so force set it to empty here.
set(CMAKE_C_LINK_FLAGS "")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-Map,\"RTOSDemo.map\" -Wl,-T\"${BOARD_FILES_DIR}/source/sys_link.ld\"")

# Debug
message("Project: ${PROJECT_NAME}")
message("Demo Sources: ${FREERTOS_DEMO_SOURCES}")
message("FreeRTOS Sources: ${FREERTOS_KERNEL_SOURCES}")
message("Port Sources: ${FREERTOS_PORT_SOURCES}")

# Create Blinky Demo executable
add_executable(RM46_FreeRTOS_Blinky_Demo.out
    ${FREERTOS_DEMO_SOURCES}
    ${FREERTOS_KERNEL_SOURCES}
    ${FREERTOS_PORT_SOURCES}
    ${COMMON_DEMO_TASKS_DIR}/MPU_Demo/mpu_demo.c
    ./source/main_blinky.c
    ./source/main.c
)

# Create Full Demo executable
add_executable(RM46_FreeRTOS_Full_Demo.out
    ${FREERTOS_DEMO_SOURCES}
    ${FREERTOS_KERNEL_SOURCES}
    ${FREERTOS_PORT_SOURCES}
    ${COMMON_DEMO_TASKS_DIR}/MPU_Demo/mpu_demo.c
    ./source/main_full.c
    ./source/reg_test_GCC.S
    ./source/main.c
)

# Create MPU Demo executable
add_executable(RM46_FreeRTOS_MPU_Demo.out
    ${FREERTOS_DEMO_SOURCES}
    ${FREERTOS_KERNEL_SOURCES}
    ${FREERTOS_PORT_SOURCES}
    ${COMMON_DEMO_TASKS_DIR}/MPU_Demo/mpu_demo.c
    ./source/main.c
)

set_target_properties(RM46_FreeRTOS_Blinky_Demo.out PROPERTIES COMPILE_DEFINITIONS "mainDEMO_TYPE=1")
set_target_properties(RM46_FreeRTOS_MPU_Demo.out PROPERTIES COMPILE_DEFINITIONS "mainDEMO_TYPE=2")
set_target_properties(RM46_FreeRTOS_Full_Demo.out PROPERTIES COMPILE_DEFINITIONS "mainDEMO_TYPE=3")
