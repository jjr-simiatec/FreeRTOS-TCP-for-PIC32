# FreeRTOS-TCP-for-PIC32

A FreeRTOS+TCP port for PIC32 microcontrollers.

(Preliminary Documentation)

## Project structure

`$`  
`+---FAT`  
`|   \---nbproject`  
`+---include`  
`|   +---MX_795`  
`|   +---MZ_ECM`  
`|   \---MZ_EFM`  
`+---RTOS`  
`|   \---nbproject`  
`+---Science`  
`|   \---nbproject`  
`\---TCPIP`  
`    \---nbproject`  

## Requirements

To use the source code as-is, you need one of the following Starter Kits:

PIC32 Ethernet Starter Kit, Microchip part no. DM320004.  
PIC32MZ EC Starter Kit w/ Crypto Engine, Microchip part no. DM320006-C.  
PIC32MZ Embedded Connectivity with FPU (EF) Start Kit (Crypto), Microchip part no. DM320007-C.  

[Note: the source code should compile and run on the non-crypto variants of the PIC32MZ but project settings will need to be changed.]

MPLAB X IDE version 3.40 or later.  
MPLAB XC32 Compiler version 1.42 or later.  
FreeRTOS source code version 9.0.0.  
FreeRTOS+TCP and FreeRTOS+FAT source code version 160919.  

## Non-requirements

You don't need MPLAB Harmony, the Microchip Legacy Peripheral Libraries or Microchip Libraries for Applications.

## How to build

1. In the root folder (marked `$` in the project structure above), you will need to create symlinks to the locations of the FreeRTOS and FreeRTOS+Plus source tree. For example on Windows:  
`mklink /d FreeRTOS "%USERPROFILE%\Documents\FreeRTOSv9.0.0\FreeRTOS\Source"`  
`mklink /d FreeRTOS-Plus "%USERPROFILE%\Documents\FreeRTOS_Labs_160823\FreeRTOS-Plus\Source"`  

2. Using MPLABX IDE, open the projects FAT, RTOS and TCPIP.

3. For each project, select the required configuration and then build. The available configurations are MZ_EFM, MZ_ECM and MX_795 corresponding to the microcontroller type on each Starter Kit.

4. Finally, load the project called Science. Again, select the required configuration and build. Use the Run command in the IDE to program the microcontroller.

## How to use

The program presents a terminal interface on UART 2. The PIC32 EF Starter Kit includes a MCP2221 USB to UART converter chip so you're good to go straight away. For the other kits, you would need to connect a RS232 line driver chip using an expansion board.

The terminal interface was mainly used for testing and debugging so there are statistics displays and register dumps (which should give you an idea of the sort of things that weren't working at various points in time). When the driver began to work, it was used for experimentation and investigation.

The Packet Test task was used to evaluate performance and latency. The LEDs are toggled at various points in time. By using a scope, you can get an idea of how the stack and the hardware are performing. For example, if one Starter Kit is configured as the transmitter and another as a receiver, you can probe LED1 on the transmitter to see when the timer interrupt triggered and probe LED3 on the receiver to see when the packet arrived.

The web server part of the FreeRTOS+TCP demo also runs on the MZ class microcontrollers. In order to shoehorn a RAM disk into the 512K available, I hacked `ff_format.c` to allow the RAM disk to be formatted with FAT12. This means the `vCreateAndVerifyExampleFiles()` test from the FreeRTOS+TCP demo is able to work properly, and by extension the web server too.

## Ethernet driver

The driver consists of the following files:

`Ethernet.h`  
`Ethernet_ISR.S`  
`NetworkInterface.c`  
`PHY_isr.S`  
`PHYGeneric.c`  
`PHYGeneric.h`  

You will likely need to create a small driver for the PHY you use. Two drivers are provided, one for the DP83848 PHY used in the MX Starter Kit and one for the LAN8740A PHY used in the MZ Starter Kits. Note that this implementation assumes the PHY interrupt line is connected to the microcontroller to detect link state change events.

The following configuration parameters are available:

`ipconfigPIC32_TX_DMA_DESCRIPTORS` - Number of transmit DMA descriptors  
`ipconfigPIC32_RX_DMA_DESCRIPTORS` - Number of receive DMA descriptors  
`ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ` - Maximum MDC clock speed allowed by the PHY. Defaults to 2.5 MHz  
`ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ` - For the MZ, it should be set to the frequency of TPBCLK5  
