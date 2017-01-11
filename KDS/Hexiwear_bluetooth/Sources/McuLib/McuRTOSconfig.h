#ifndef __McuRTOS_CONFIG_H
#define __McuRTOS_CONFIG_H

/* -------------------------------------------------------------------- */
/* Macros to identify the compiler used: */
#define configCOMPILER_ARM_GCC                    1 /* GNU ARM gcc compiler */
#define configCOMPILER_ARM_IAR                    2 /* IAR ARM compiler */
#define configCOMPILER_ARM_FSL                    3 /* Legacy Freescale ARM compiler */
#define configCOMPILER_ARM_KEIL                   4 /* ARM/Keil compiler */
#define configCOMPILER_S08_FSL                    5 /* Freescale HCS08 compiler */
#define configCOMPILER_S12_FSL                    6 /* Freescale HCS12(X) compiler */
#define configCOMPILER_CF1_FSL                    7 /* Freescale ColdFire V1 compiler */
#define configCOMPILER_CF2_FSL                    8 /* Freescale ColdFire V2 compiler */
#define configCOMPILER_DSC_FSL                    9 /* Freescale DSC compiler */

#define configCOMPILER                            configCOMPILER_ARM_GCC
/* -------------------------------------------------------------------- */
/* CPU family identification */
#define configCPU_FAMILY_S08                      1   /* S08 core */
#define configCPU_FAMILY_S12                      2   /* S12(X) core */
#define configCPU_FAMILY_CF1                      3   /* ColdFire V1 core */
#define configCPU_FAMILY_CF2                      4   /* ColdFire V2 core */
#define configCPU_FAMILY_DSC                      5   /* 56800/DSC */
#define configCPU_FAMILY_ARM_M0P                  6   /* ARM Cortex-M0+ */
#define configCPU_FAMILY_ARM_M4                   7   /* ARM Cortex-M4 */
#define configCPU_FAMILY_ARM_M4F                  8   /* ARM Cortex-M4F (with floating point unit) */
#define configCPU_FAMILY_ARM_M7                   9   /* ARM Cortex-M7 */
#define configCPU_FAMILY_ARM_M7F                  10  /* ARM Cortex-M7F (with floating point unit) */
/* Macros to identify set of core families */
#define configCPU_FAMILY_IS_ARM_M0(fam)           ((fam)==configCPU_FAMILY_ARM_M0P)
#define configCPU_FAMILY_IS_ARM_M4(fam)           (((fam)==configCPU_FAMILY_ARM_M4)  || ((fam)==configCPU_FAMILY_ARM_M4F))
#define configCPU_FAMILY_IS_ARM_M7(fam)           (((fam)==configCPU_FAMILY_ARM_M7)  || ((fam)==configCPU_FAMILY_ARM_M7F))
#define configCPU_FAMILY_IS_ARM_M4_M7(fam)        (configCPU_FAMILY_IS_ARM_M4(fam) || configCPU_FAMILY_IS_ARM_M7(fam))
#define configCPU_FAMILY_IS_ARM_FPU(fam)          (((fam)==configCPU_FAMILY_ARM_M4F) || ((fam)==configCPU_FAMILY_ARM_M7F))
#define configCPU_FAMILY_IS_ARM(fam)              (configCPU_FAMILY_IS_ARM_M0(fam) || configCPU_FAMILY_IS_ARM_M4(fam) || configCPU_FAMILY_IS_ARM_M7(fam))

#define configCPU_FAMILY                          configCPU_FAMILY_ARM_M0P

/*-----------------------------------------------------------
 * GDB backtrace handler support
 * See http://interactive.freertos.org/entries/23468301-Tasks-backtrace-switcher-viewer-snippet-for-debugger-gcc-gdb-ARM-Cortex-M3-MPU-port-Eclipse-support-
 *----------------------------------------------------------*/
#define configGDB_HELPER                          (0 && configCPU_FAMILY_IS_ARM(configCPU_FAMILY) && (configCOMPILER==configCOMPILER_ARM_GCC)) /* 1: enable special GDB stack backtrace debug helper; 0: disabled */


#endif /* __McuRTOS_CONFIG_H */
