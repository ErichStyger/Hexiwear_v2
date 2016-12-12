#ifndef __MCUC1CONFIG_H
#define __MCUC1CONFIG_H

/* Identifiers used to identify the SDK */
#define MCUC1_CONFIG_SDK_PROCESSOR_EXPERT    1 /* using Processor Expert SDK */
#define MCUC1_CONFIG_SDK_KINETIS_1_3         2 /* using NXP Kinetis SDK V1.3 */
#define MCUC1_CONFIG_SDK_KINETIS_2_0         3 /* using NXP Kinetis SDK V2.0 */
#define MCUC1_CONFIG_SDK_MCUXPRESSO_2_0      4 /* same as Kinetis SDK v2.0 */

/* identify SDK used */
#ifndef MCUC1_CONFIG_SDK_VERSION_USED
  #define MCUC1_CONFIG_SDK_VERSION_USED  MCUC1_CONFIG_SDK_KINETIS_2_0
#endif

#endif /* __MCUC1CONFIG_H */

