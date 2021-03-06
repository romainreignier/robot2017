/*
    ChibiOS/RT - Copyright (C) 2016

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef MCUCONF_COMMUNITY_H
#define MCUCONF_COMMUNITY_H

/*
 * FSMC SRAM driver system settings.
 */
#define STM32_USE_FSMC_SRAM                 FALSE

/*
 * QEI driver system settings.
 */
#define STM32_QEI_USE_TIM1                FALSE
#define STM32_QEI_USE_TIM2                TRUE
#define STM32_QEI_USE_TIM3                TRUE
#define STM32_QEI_USE_TIM5                FALSE
#define STM32_QEI_USE_TIM8                FALSE
#define STM32_QEI_TIM1_IRQ_PRIORITY         7
#define STM32_QEI_TIM2_IRQ_PRIORITY         7
#define STM32_QEI_TIM3_IRQ_PRIORITY         7
#define STM32_QEI_TIM5_IRQ_PRIORITY         7
#define STM32_QEI_TIM8_IRQ_PRIORITY         7

#endif /* MCUCONF_COMMUNITY_H */
