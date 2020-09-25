# FreeRTOS-TCP-for-PIC32

A FreeRTOS+TCP port for PIC32 microcontrollers.

## Project structure
```
$
+---CLI
|   \---nbproject
+---FAT
|   \---nbproject
+---include
|   +---MX_795
|   +---MZ_DAB
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

  * PIC32 Ethernet Starter Kit, Microchip part no. DM320004.
  * PIC32MZ EC Starter Kit w/ Crypto Engine, Microchip part no. DM320006-C.
  * PIC32MZ Embedded Connectivity with FPU (EF) Start Kit (Crypto), Microchip part no. DM320007-C.
  * PIC32MZ Embedded Graphics with External DRAM (DA) Starter Kit (Crypto), Microchip part no. DM320008-C.

The Ethernet module is the same across the PIC32 range and this driver has been verified to work on PIC32MX795F512L, PIC32MZ2048ECM144, PIC32MZ2048EFM144 and PIC32MZ2064DAB288. The supported Ethernet PHYs include DP83848, LAN8740A and LAN9303.

[**Note**: the source code should compile and run on the non-crypto variants of the PIC32MZ Starter Kits but project settings will need to be changed and `PIC32_HAS_WORKING_TRNG` will need to be undefined.]

[**Note**: the replacement for DM320004, Microchip part no. DM320004-2, has minimal changes to the circuits but does use a different PHY from the original.]

The following software and source code packages are required:
  * MPLAB X IDE version 5.40 or later.
  * MPLAB XC32 Compiler version 2.41 or later.
  * FreeRTOS source code version 10.4.0.
  * FreeRTOS Labs source code version 200218.

[**Note**: there are fixes for FreeRTOS+TCP and FreeRTOS+FAT that have not been released officially yet. See the known issues section for more info.]

## Non-requirements

You don't need MPLAB Harmony, the Microchip Legacy Peripheral Libraries or Microchip Libraries for Applications.

## How to build

1. In the root folder (marked `$` in the project structure above), you will need to copy/unpack the FreeRTOS and FreeRTOS-Labs source code. Alternatively, create symbolic links to the locations of the source trees. For example on Windows:  
`mklink /d FreeRTOS "%USERPROFILE%\Documents\FreeRTOS\FreeRTOS"`  
`mklink /d FreeRTOS-Plus "%USERPROFILE%\Documents\FreeRTOS\FreeRTOS-Plus"`  
`mklink /d FreeRTOS-Labs "%USERPROFILE%\Documents\FreeRTOS-Labs\FreeRTOS-Labs"`

2. Using MPLABX IDE, open the projects `CLI`, `FAT`, `RTOS` and `TCPIP`.

3. For each project, select the required configuration and then build. The available configurations are `MZ_DAB`, `MZ_EFM`, `MZ_ECM` and `MX_795` corresponding to the microcontroller type on each Starter Kit.

4. Finally, load the project called `Science`. Again, select the required configuration and build. Use the Run command in the IDE to program the microcontroller.

## Known issues

A couple of bugs in FreeRTOS+TCP and FreeRTOS+FAT can result in heap corruption or an exception. Use unoffical code releases that can be found here: https://sourceforge.net/p/freertos/discussion/382005/thread/c27aeb20b5/ and also manually apply the modification described here: https://sourceforge.net/p/freertos/discussion/382005/thread/6e3856e518/

Compiler versions later than v1.44 would sometimes output the assembler code in `PIC32Arch.h` incorrectly (bad register allocations) depending on optimisation settings etc. This accounts for reports that the driver was not working. A modification to the constraints seems to have cured this problem for v2.20 but has only been tested using `-O0` and `-O1`.

The FTP and web server are dependent on FreeRTOS+FAT. If you don't need that functionality, you can eliminate the dependency by excluding the source files for those components in the `TCPIP` project.

## How to use

The program presents a terminal interface via UART2 and a command line interface on TCP port 12345. The PIC32 EF Starter Kit includes a MCP2221 USB to UART converter chip so you can use either interface. For the other kits, you can only use the TCP CLI unless you're willing to add a RS232 interface via the expansion connector.

The UART terminal interface was mainly used for testing and debugging so there are statistics displays and register dumps (which should give you an idea of the sort of things that weren't working at various points in time). When the driver began to work, it was used for experimentation and investigation.

The Packet Test task was used to evaluate performance and latency. The LEDs are toggled at various points in time. By using a scope, you can get an idea of how the stack and the hardware are performing. For example, if one Starter Kit is configured as the transmitter and another as a receiver, you can probe LED1 on the transmitter to see when the timer interrupt triggered and probe LED3 on the receiver to see when the packet arrived.

The web server and FTP server parts of the FreeRTOS+TCP demo will also run on the PIC32. The web server demo files are stored in a blob containing a FAT file system prepared using Linux. This blob resides in Flash and is mounted in `/www`. I hacked `ff_format.c` to allow the RAM disk to be formatted with FAT12. This allows a small RAM disk to be created that will pass the `vCreateAndVerifyExampleFiles()` test from the FreeRTOS+TCP demo. The RAM disk is mounted in `/ram`. This arrangement reduces RAM usage considerably and allows the demo to work on the MX class microcontroller too.

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
Unless you are using an already supported PHY, you will need to create a small driver. Three drivers are provided: one for the DP83848 PHY used in the MX Starter Kit, one for the LAN8740A PHY used in the MZ Starter Kits and a simple LAN9303 driver. Note that this implementation assumes the PHY interrupt line is connected to the microcontroller to detect link state change events. PHY support is needed to use Wake-on-LAN or the Ethernet cable diagnostics - only the LAN8740A has all of these features. All PHYs allow the interface to be brought up and down (powered off).

The following configuration parameters are available. Values without defaults must be configured:

`ipconfigPIC32_TX_DMA_DESCRIPTORS` - number of transmit DMA descriptors, defaults to 10  
`ipconfigPIC32_RX_DMA_DESCRIPTORS` - number of receive DMA descriptors, defaults to 20  
`ipconfigPIC32_MIIM_MANAGEMENT_MAX_CLK_HZ` - maximum MDC clock speed allowed by the PHY, defaults to 2.5 MHz  
`ipconfigPIC32_MIIM_SOURCE_CLOCK_HZ` - for the MZ __only__: frequency of T<sub>PBCLK5</sub>  
`ipconfigPIC32_DRV_TASK_PRIORITY` - driver task priority  
`ipconfigPIC32_DRV_TASK_STACK_SIZE` - driver task stack size in words  
`ipconfigPIC32_ETH_INT_PRIORITY` - Ethernet controller interrupt priority  
`ipconfigPIC32_DRV_TASK_BLOCK_TICKS` - maximum time the driver waits for stack resources, defaults to portMAX_DELAY

If you want to use one of the built-in PHY drivers, you can set the following configuration parameter:

`ipconfigPIC32_PHY_DRIVER` - One of: `PIC32_PHY_DRIVER_DP83848`, `PIC32_PHY_DRIVER_LAN8740A`, `PIC32_PHY_DRIVER_LAN9303`

The following configuration parameters describe the hardware configuration of your PHY:

`ipconfigPIC32_PHY_ADDRESS` - PHY MII address  
`ipconfigPIC32_PHY_INTERRUPT_VECTOR` - PIC32 interrupt vector for the PHY interrupt  
`ipconfigPIC32_PHY_ASSERT_HW_RESET()` - Optional code to put the PHY in reset  
`ipconfigPIC32_PHY_CLEAR_HW_RESET()` - Optional code to release the PHY from reset  
`ipconfigPIC32_PHY_ENABLE_INTERRUPT()` - Prepare the PIC32 to accept PHY interrupts  
`ipconfigPIC32_PHY_DISABLE_INTERRUPT()` - Mask PHY interrupts  
`ipconfigPIC32_PHY_CLEAR_INTERRUPT()` - Acknowledge the PHY interrupt after processing  

## Other experiments

As well as testing the driver, the test harness demonstrates the following additional features:
  * Wake on LAN;
  * Programmatic Ethernet interface up/down;
  * SRAM/Flash wait state configuration for the PIC32MX;
  * DDR2 controller initialisation for the PIC32MZ DA;
  * Built-in tests: March C- RAM test, Ethernet loopback, Ethernet cable diagnostic.
