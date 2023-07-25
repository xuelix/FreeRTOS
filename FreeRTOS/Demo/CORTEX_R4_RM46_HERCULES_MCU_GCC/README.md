This directory contains a FreeRTOS project to build either a blinky, full demo, or MPU demo for the RM46L852 Target. 
It is set up to blink LEDs on the Texas Instruments LAUNCHXL2-RM46 and the TMDXRM46HDK Development Kits.

The code related to the main files lives in source
The code related to the board setup lives in BoardFiles

This demo can either be loaded into Texas Instrument's Code Composer Studio (CCS) or built using CMAKE. 
Please be aware there is a filter on CMakeLists.txt and the "build" directory in the CCS project. 
This is to keep CCS from attempting to use resources generated with a CMAKE build.
If a directory other than "build" is selected when building using CMAKE CCS will attempt to use the the files in that directory, leading to build issues in CCS.
At time of writing this can be fixed by right clicking the folder in CCS and selecting "Exclude from build".

For CMAKE the build command there are options to build either a Blinky, Full, or MPU Demo
The three targets are RM46_FreeRTOS_Blinky_Demo.out, RM46_FreeRTOS_Full_Demo.out, RM46_FreeRTOS_MPU_Demo.out binaries
The all options builds both of these.
Example Usage:
    cmake -S . -B build;
    make -C build all;

The generated binaries can then be found in the build directory.
This can be flashed onto the board through either using uniflash or by using CCS.

UART output is available by opening a Serial Connection to the board. 
The settings for it are a BAUD rate of 115200, 1 stopbit, and None Parity

