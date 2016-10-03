#ifndef FREERTOS_CONFIG_MZ_EFM_H
#define FREERTOS_CONFIG_MZ_EFM_H

#if !defined(__32MZ2048EFM144__)
#error "This file is for the PIC32MZ EFM"
#endif

#define configCPU_CLOCK_HZ                              ( 200000000UL )

#ifndef __MPLAB_DEBUGGER_SIMULATOR
#define configPERIPHERAL_CLOCK_HZ                       ( 40000000UL )
#endif

/* Enable support for Task based FPU operations. This will enable support for
FPU context saving during switches only on architectures with hardware FPU.

NOTE: This constant is defined in the project options as configurations are 
provided that both enable and disable floating point support. */
#define configUSE_TASK_FPU_SUPPORT                      1

#endif /* FREERTOS_CONFIG_MZ_EFM_H */
