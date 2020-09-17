/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "pins_arduino.h"
// Pin number
constexpr PinName digitalPin[] = {
    PA_0,  //D0
    PA_1,  //D1
    PA_2,  //D2
    PA_3,  //D3
    PA_4,  //D4
    PA_5,  //D5
    PA_6,  //D6
    PA_7,  //D7
    PA_8,  //D8
    PA_9,  //D9
    PA_10, //D10
    PA_11, //D11
    PA_12, //D12
    PA_13, //D13
    PA_14, //D14
    PA_15, //D15
    PB_0,  //D16
    PB_1,  //D17
    PB_2,  //D18
    PB_3,  //D19
    PB_4,  //D20
    PB_5,  //D21
    PB_6,  //D22
    PB_7,  //D23
    PB_8,  //D24
    PB_9,  //D25
    PB_10, //D26
    PB_11, //D27
    PB_12, //D28
    PB_13, //D29
    PB_14, //D30
    PB_15, //D31
    PC_0,  //D32
    PC_1,  //D33
    PC_2,  //D34
    PC_3,  //D35
    PC_4,  //D36
    PC_5,  //D37
    PC_6,  //D38
    PC_7,  //D39
    PC_8,  //D40
    PC_9,  //D41
    PC_10, //D42
    PC_11, //D43
    PC_12, //D44
    PC_13, //D45
    PC_14, //D46
    PC_15, //D47
    PD_0,  //D48
    PD_1,  //D49
    PD_2   //D50
};

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
 
/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {};

    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /* Initializes the CPU, AHB and APB busses clocks */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    // moses notes:
    // Changed APB1 CLK divider from DIV2 to DIV1
    // Changed Flash latency from 2 to 1. (Appears stable, noticable improvement)
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    uint32_t latency = FLASH_LATENCY_1;
    if(RCC_OscInitStruct.PLL.PLLMUL > RCC_PLL_MUL10) {
        latency = FLASH_LATENCY_2; // overclocking, need more waitstates.
    }
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, latency) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}
//

#ifdef __cplusplus
}
#endif
