/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "i2c.h"  // Include I2C header
#include "zlg7290.h" // Include ZLG7290 header
#include "tim.h"
#include "Steering_Engine.h"
#include "Dc_motor.h"
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//uint32_t adcx[4]={0};
__IO uint16_t adcx[4]={0}; // ADC raw values buffer
float temp0,temp1,temp2,temp3; // Temporary variables for sensor values
// Variable to store the light sensor value
float light_value;
float servoAngle;
#define ZLG_READ_ADDRESS1         0x01 // ZLG7290 Key value read register address
#define ZLG_READ_ADDRESS2         0x10 // ZLG7290 Display data read start address (possibly unused)
#define ZLG_WRITE_ADDRESS1        0x10 // ZLG7290 Display data write start address
#define ZLG_WRITE_ADDRESS2        0x11 // ZLG7290 Display data write second byte start address (for specific multi-digit display operations)
#define countof(a) (sizeof(a) / sizeof(*(a))) // Macro to calculate the number of elements in an array

uint8_t Tx1_Buffer[8]={0}; // Buffer to store data to be sent to ZLG7290 display

const uint8_t segment_codes[10] = { // Segment codes for digits 0-9 for ZLG7290 (common cathode assumed)
    0xFC, // 0 (abcdef)
    0x60, // 1 (bc) - Note: 0x0C in 9_ZLG7290's switch_flag might be 'L'
    0xDA, // 2 (abdeg)
    0xF2, // 3 (abcdg)
    0x66, // 4 (bc fg)
    0xB6, // 5 (acdfg)
    0xBE, // 6 (acdefg)
    0xE0, // 7 (abc)
    0xFE, // 8 (abcdefg)
    0xE6  // 9 (abcdfg)
};

#define SEG_DP    0x01  // Assume DP is bit0 (LSB), active high. Please verify!
                      // If DP is bit7 (MSB), it would be 0x80
#define SEG_BLANK 0x00  // Segment code for blank/off

uint8_t marquee_count = 0; // Counter for marquee LED sequence

volatile uint8_t g_buzzer_should_sound = 0; // Flag to control buzzer state
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void display_light_value_on_zlg7290(float value);
void Turn_On_Marquee_LED(uint8_t led_index); // Function to turn on a specific marquee LED
void Turn_Off_All_Marquee_LEDs(void);    // Function to turn off all marquee LEDs
void beer_should_sound(void); // Function to control buzzer sound state
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void beer_should_sound(void)
{
    if (g_buzzer_should_sound > 0)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET); // Turn on buzzer
        g_buzzer_should_sound--; // Decrement remaining ticks
    }
    else
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // Turn off buzzer
    }
}

/**
  * @brief  Converts the light value to segment codes and prepares it for ZLG7290 display.
  * @param  value: The light value as a float (e.g., 5.2368).
  * @retval None
  * Tx1_Buffer[0] corresponds to the leftmost digit (DIG1), Tx1_Buffer[7] to the rightmost (DIG8).
  * Display format is XX.YY (e.g., 05.23) using DIG4, DIG5 (with DP), DIG6, DIG7.
  * DIG1, DIG2, DIG3, DIG8 will be blank.
  */
void display_light_value_on_zlg7290(float value)
{
    int val_int_part; // Integer part of the value (not used in current logic)
    int val_frac_part; // Fractional part of the value (not used in current logic)
    uint8_t digit1, digit2, digit3, digit4; // XX.YY -> d1 d2 . d3 d4

    // Initialize all segments to blank
    for (int i = 0; i < 8; i++) {
        Tx1_Buffer[i] = SEG_BLANK;
    }

    // Clamp the value to the displayable range 0.00 to 99.99
    if (value < 0.0f) value = 0.0f;
    if (value > 99.995f) value = 99.995f; // For rounding to 99.99

    // Scale the value by 100 and round to the nearest integer (e.g., 5.2368 -> 523.68 -> 524)
    int scaled_value = (int)(value * 100.0f + 0.5f);

    // Extract individual digits for XX.YY format
    digit1 = (scaled_value / 1000) % 10; // Tens digit (XX)
    digit2 = (scaled_value / 100) % 10;  // Ones digit (XX)
    digit3 = (scaled_value / 10) % 10;   // Tenths digit (YY)
    digit4 = scaled_value % 10;          // Hundredths digit (YY)

    // Display mapping: _ _ D1 D2 . D3 D4 _
    // Tx1_Buffer[2] = digit1 (tens)
    // Tx1_Buffer[3] = digit2 (ones) + decimal point
    // Tx1_Buffer[4] = digit3 (tenths)
    // Tx1_Buffer[5] = digit4 (hundredths)

    // Handle leading zero for values less than 10 (e.g., 05.23)
    if (digit1 == 0 && value < 10.0f) {
        Tx1_Buffer[2] = SEG_BLANK; // Or segment_codes[0] to display leading '0'
    } else {
        Tx1_Buffer[2] = segment_codes[digit1];
    }
    
    Tx1_Buffer[3] = segment_codes[digit2] | SEG_DP; // Ones digit with decimal point
    Tx1_Buffer[4] = segment_codes[digit3];          // Tenths digit
    Tx1_Buffer[5] = segment_codes[digit4];          // Hundredths digit
    
    // Example for 3-digit display X.Y (e.g., light value 5.23 -> 5.2)
    // scaled_value = (int)(value * 10.0f + 0.5f); // 5.23 -> 52
    // digit1 = (scaled_value / 10) % 10; // Ones digit
    // digit2 = scaled_value % 10;       // Tenths digit
    // Tx1_Buffer[3] = segment_codes[digit1] | SEG_DP;
    // Tx1_Buffer[4] = segment_codes[digit2];
}

// Function to turn on a specific marquee LED
// Assumes active-low LEDs (setting pin LOW turns LED ON)
void Turn_On_Marquee_LED(uint8_t led_index)
{
    switch(led_index)
    {
        case 0:
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_RESET); /* Light up D4 (PH15) */
            break;
        case 1:
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); /* Light up D3 (PB15) */
            break;
        case 2:
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);  /* Light up D2 (PC0) */
            break;
        case 3:
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET); /* Light up D1 (PF10) */
            break;
        default:
            // Optional: Turn off all if index is out of bounds
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
            break;
    }
}

// Function to turn off all marquee LEDs
// Assumes active-low LEDs (setting pin HIGH turns LED OFF)
void Turn_Off_All_Marquee_LEDs(void)
{
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_15, GPIO_PIN_SET); // Turn off D4 (PH15)
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // Turn off D3 (PB15)
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);  // Turn off D2 (PC0)
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET); // Turn off D1 (PF10)
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_TIM12_Init(); // init PWM timer
  /* USER CODE BEGIN 2 */
  MX_I2C1_Init();
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)adcx,4); // Start ADC conversion with DMA
  Turn_Off_All_Marquee_LEDs(); // Ensure marquee LEDs are off at startup
  /* USER CODE END 2 */
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); 
  SteeringEngine_RotateFullCircle();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

    temp0 = (float)adcx[0]*(3.3/4096); // ADC Channel 0
    temp1 = (float)adcx[1]*(3.3/4096); // ADC Channel 1 (e.g., light sensor)
    temp2 = (float)adcx[2]*(3.3/4096); // ADC Channel 2
    temp3 = (float)adcx[3]*(3.3/4096); // ADC Channel 3
    light_value = temp1*10; // Scale light sensor value (example scaling)
    servoAngle = light_value * (180.0/33.0); // function to getangle
    printf("\r Light Sensor Value =%f\r",light_value); // Print light sensor value via UART
    printf("\r\n servoAngle = %f\r\n", servoAngle); // Print servo angle
    display_light_value_on_zlg7290(light_value); // Prepare display data for ZLG7290
    // Write data from Tx1_Buffer to ZLG7290 display via I2C
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8);
    SteeringEngine_Rotate(servoAngle); // Rotate steering engine based on light value
    // Marquee LED control based on light_value
    if (light_value > 20.0f)
    {
        Turn_Off_All_Marquee_LEDs(); // Turn off all LEDs before lighting the next one in sequence
        Turn_On_Marquee_LED(marquee_count % 4); // Turn on the current LED in sequence
        marquee_count++; // Move to the next LED for the next cycle
        if (g_buzzer_should_sound == 0) { // If not already beeping or scheduled to beep
            g_buzzer_should_sound = 1000;  // Set to beep for 100 SysTick interrupts (approx 100ms)
        }
        DC_Task(0x13); // Rotate DC motor (example command)
    }
    else
    {
        Turn_Off_All_Marquee_LEDs(); // Turn off all marquee LEDs if light value is not greater than 20
        g_buzzer_should_sound = 0; // Stop any ongoing/scheduled beep immediately
        DC_Task(0x00); // Stop DC motor (example command)
    }

    HAL_Delay(500); // Delay for 1 second
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
// Redirect fputc to USART1 for printf
int fputc(int ch, FILE *f)
{ 	
    while((USART1->SR&0X40)==0); // Wait for USART1 Tx buffer to be empty
    USART1->DR = (uint8_t) ch;      
    return ch;
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/