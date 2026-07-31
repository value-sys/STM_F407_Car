# DAPLink flashing (STM32F407)

This project is configured for the **STM32F407VET6**, not the STM32H7 used by
the reference project. OpenOCD loads `target/stm32f4x.cfg` through
`OpenOCD/daplink_stm32f407.cfg`.

## Flash firmware

1. Connect DAPLink to the board: SWDIO, SWCLK, GND, and VTref/3.3 V.
2. Build the `Debug` CMake preset so that
   `build/Debug/Tacking_car.elf` exists.
3. Double-click `DAPLink_Flash.bat`.

The script programs the ELF, verifies it, resets the STM32F407, and exits.
You can also drag another `.elf` file onto the BAT to flash that file.

## Debug with Ozone or GDB

1. Double-click `DAPLink_OpenOCD.bat` and keep its terminal open.
2. Select an external GDB server in the debugger and connect to
   `localhost:3333`.
3. Load `build/Debug/Tacking_car.elf` as the program and debug-information
   file.

Ozone must be version 3.40 or newer for its external GDB Server mode. The
OpenOCD terminal must report a CMSIS-DAP probe and an STM32F4 target before a
hardware debug session can start.

Both BAT files derive all paths from their own project directory. No absolute
path needs to be edited after the project is moved.
