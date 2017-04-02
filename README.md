# FreeRTOS-TCP-for-PIC32

A FreeRTOS+TCP port for PIC32 microcontrollers.

(Preliminary Documentation)

## Project structure
```
$
+---CLI
|   \---nbproject
+---FAT
|   \---nbproject
+---include
|   +---MX_795
|   +---MZ_ECM
|   \---MZ_EFM
+---RTOS
|   \---nbproject
+---Science
|   \---nbproject
\---TCPIP
    \---nbproject
```
## Requirements
To use the source code as-is, you need one of the following Starter Kits:

PIC32 Ethernet Starter Kit, Microchip part no. DM320004.  
PIC32MZ EC Starter Kit w/ Crypto Engine, Microchip part no. DM320006-C.  
PIC32MZ Embedded Connectivity with FPU (EF) Start Kit (Crypto), Microchip part no. DM320007-C.  

[**Note**: the source code should compile and run on the non-crypto variants of the PIC32MZ Starter Kits but project settings will need to be changed.]

[**Note**: the replacement for DM320004, Microchip part no. DM320004-2, has minimal changes to the circuits but does use a different PHY from the original.]

MPLAB X IDE version 3.40 or later.  
MPLAB XC32 Compiler version 1.42 or later.  
FreeRTOS source code version 9.0.0.  
FreeRTOS+TCP and FreeRTOS+FAT source code version 160919.  

## Non-requirements

You don't need MPLAB Harmony, the Microchip Legacy Peripheral Libraries or Microchip Libraries for Applications.

## How to build

1. In the root folder (marked `$` in the project structure above), you will need to copy/unpack the FreeRTOS and FreeRTOS-Plus source code. Alternatively, create symbolic links to the locations of the FreeRTOS and FreeRTOS-Plus source trees. For example on Windows:  
`mklink /d FreeRTOS "%USERPROFILE%\Documents\FreeRTOSv9.0.0\FreeRTOS"`  
`mklink /d FreeRTOS-Plus "%USERPROFILE%\Documents\FreeRTOS_Labs_160919\FreeRTOS-Plus"`

2. Using MPLABX IDE, open the projects `CLI`, `FAT`, `RTOS` and `TCPIP`.

3. For each project, select the required configuration and then build. The available configurations are `MZ_EFM`, `MZ_ECM` and `MX_795` corresponding to the microcontroller type on each Starter Kit.

4. Finally, load the project called `Science`. Again, select the required configuration and build. Use the Run command in the IDE to program the microcontroller.

## How to use

The program presents a terminal interface via UART2 and a command line interface on TCP port 12345. The PIC32 EF Starter Kit includes a MCP2221 USB to UART converter chip so you can use either interface. For the other kits, you can only use the TCP CLI unless you're willing to add a RS232 interface via the expansion connector.

The UART terminal interface was mainly used for testing and debugging so there are statistics displays and register dumps (which should give you an idea of the sort of things that weren't working at various points in time). When the driver began to work, it was used for experimentation and investigation.

The Packet Test task was used to evaluate performance and latency. The LEDs are toggled at various points in time. By using a scope, you can get an idea of how the stack and the hardware are performing. For example, if one Starter Kit is configured as the transmitter and another as a receiver, you can probe LED1 on the transmitter to see when the timer interrupt triggered and probe LED3 on the receiver to see when the packet arrived.

The web server and ftp server parts of the FreeRTOS+TCP demo will also run on the PIC32. The web server demo files are stored in a blob containing a FAT file system prepared using Linux. This blob resides in Flash and is mounted in `/www`. I hacked `ff_format.c` to allow the RAM disk to be formatted with FAT12. This allows a small RAM disk to be created that will pass the `vCreateAndVerifyExampleFiles()` test from the FreeRTOS+TCP demo. The RAM disk is mounted in `/ram`. This arrangement reduces RAM usage considerably and allows the demo to work on the MX class microcontroller too.

## Ethernet driver

The driver consists of the following files:
```
Ethernet.h
Ethernet_ISR.S
EthernetPrivate.h
NetworkInterface.c
PHY_isr.S
PHYGeneric.c
PHYGeneric.h
PIC32Arch.h
```
The FreeRTOS+TCP configuration defined in `FreeRTOSIPConfig.h` must contain the following directives for correct operation:
```C
#define ipconfigZERO_COPY_TX_DRIVER                 (1)
#define ipconfigZERO_COPY_RX_DRIVER                 (1)
#define ipconfigUSE_LINKED_RX_MESSAGES              (1)
#define ipconfigDRIVER_INCLUDED_TX_IP_CHECKSUM      (0)
#define ipconfigDRIVER_INCLUDED_RX_IP_CHECKSUM      (0)
#define ipconfigETHERNET_DRIVER_FILTERS_PACKETS     (0)
#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES (1)
```
You will likely need to create a small driver for the PHY you use. Two drivers are provided, one for the DP83848 PHY used in the MX Starter Kit and one for the LAN8740A PHY used in the MZ Starter Kits. Note that this implementation assumes the PHY interrupt line is connected to the microcontroller to detect link state change events.

The following configuration parameters are available. Values without defaults must be configured:

`ipconfigPIC32_TX_DMA_DESCRIPTORS` - number of transmit DMA descriptors, defaults to 10  
`ipconfigPIC32_RX_DMA_DESCRIPTORS` - number of receive DMA descriptors, defaults to 20  
`ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ` - maximum MDC clock speed allowed by the PHY, defaults to 2.5 MHz  
`ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ` - for the MZ __only__: frequency of T<sub>PBCLK5</sub>  
`ipconfigPIC32_DRV_TASK_PRIORITY` - driver task priority  
`ipconfigPIC32_DRV_TASK_STACK_SIZE` - driver task stack size in words  
`ipconfigPIC32_ETH_INT_PRIORITY` - Ethernet controller interrupt priority  
`ipconfigPIC32_DRV_TASK_BLOCK_TICKS` - maximum time the driver waits for stack resources, defaults to portMAX_DELAY  
