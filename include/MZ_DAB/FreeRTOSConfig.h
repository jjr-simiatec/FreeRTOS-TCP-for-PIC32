#ifndef FREERTOS_CONFIG_MZ_DAB_H
#define FREERTOS_CONFIG_MZ_DAB_H

#if !defined(__32MZ2064DAB288__)
#error "This file is for the PIC32MZ DAB"
#endif

#define configCPU_CLOCK_HZ                              ( 200000000UL )

#ifndef __MPLAB_DEBUGGER_SIMULATOR
#define configPERIPHERAL_CLOCK_HZ                       ( 40000000UL )
#endif

/* Enable support for Task based FPU operations. This will enable support for
FPU context saving during switches only on architectures with hardware FPU.

NOTE: This constant is defined in the project options as configurations are
provided that both enable and disable floating point support. */
#define configUSE_TASK_FPU_SUPPORT                      0

#define PIC32_HAS_WORKING_TRNG

#endif /* FREERTOS_CONFIG_MZ_DAB_H */
