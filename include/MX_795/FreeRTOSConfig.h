#ifndef FREERTOS_CONFIG_MX_795_H
#define FREERTOS_CONFIG_MX_795_H

#if !defined(__32MX795F512L__)
#error "This file is for the PIC32MX"
#endif

#define configCPU_CLOCK_HZ                              ( 80000000UL )

#ifndef __MPLAB_DEBUGGER_SIMULATOR
#define configPERIPHERAL_CLOCK_HZ                       ( 40000000UL )
#endif

#define configUSE_TASK_FPU_SUPPORT                      0

#endif /* FREERTOS_CONFIG_MX_795_H */
