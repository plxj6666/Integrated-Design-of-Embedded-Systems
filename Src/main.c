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

// Hot start data structure
#define HOT_START_FLAG_VALUE              0x12345678  // Magic number for main hot start data
#define HOT_START_VALIDATION_FLAG_VALUE   0xABCDEF01  // Magic number for validation data

typedef struct {
    uint32_t hot_start_flag;           // Should be HOT_START_FLAG_VALUE
    uint32_t light_value_raw;
    uint32_t marquee_count_saved;
    uint32_t buzzer_enabled_saved;
    uint32_t motor_state_saved;
    uint32_t checksum;
} HotStartData_t;

typedef struct {
    uint32_t validation_flag;           // Should be HOT_START_VALIDATION_FLAG_VALUE
    uint32_t light_value_raw_plus_one;
    uint32_t marquee_count_saved_plus_one;
    uint32_t buzzer_enabled_saved_plus_one;
    uint32_t motor_state_saved_plus_one;
} HotStartValidationData_t;

// Define the backup register base address for STM32F4
#define BACKUP_REG_BASE_ADDR ((uint32_t *)0x40002850)  // STM32F4 backup register base for HotStartData_t base
HotStartData_t *hot_start_data = (HotStartData_t *)BACKUP_REG_BASE_ADDR;
// Place validation data immediately after hot_start_data in backup RAM
HotStartValidationData_t *hot_start_validation_data = (HotStartValidationData_t *)((uint8_t *)BACKUP_REG_BASE_ADDR + sizeof(HotStartData_t));

uint8_t is_hot_start = 0;  // hot start flag

#define ZLG_READ_ADDRESS1         0x01 // ZLG7290 Key value read register address
#define ZLG_READ_ADDRESS2         0x10 // ZLG7290 Display data read start address (possibly unused)
#define ZLG_WRITE_ADDRESS1        0x10 // ZLG7290 Display data write start address
#define ZLG_WRITE_ADDRESS2        0x11 // ZLG7290 Display data write second byte start address (for specific multi-digit display operations)
#define countof(a) (sizeof(a) / sizeof(*(a))) // Macro to calculate the number of elements in an array

volatile uint8_t g_key_interrupt_flag = 0;
uint8_t g_raw_key_value = 0; // Stores the raw key value from ZLG7290

// Default main loop delay
uint32_t g_main_loop_delay_ms = 50;

// Feature enable flags controlled by keys
uint8_t g_marquee_logic_enabled = 0; // 0 = disabled, 1 = can be activated by light
uint8_t g_fan_logic_enabled = 0;     // 0 = disabled, 1 = can be activated by light

// Placeholder Key Codes from ZLG7290 - VERIFY AND REPLACE THESE
#define KEY_CODE_1    0x1C // Example key code for '1'
#define KEY_CODE_2    0x1B // Example key code for '2'
#define KEY_CODE_3    0x1A // Example key code for '3'
#define KEY_CODE_A    0x19 // Example key code for 'A'
#define KEY_CODE_B    0x11 // Example key code for 'B'
#define KEY_CODE_C    0x09 // Example key code for 'C'

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
volatile uint8_t g_buzzer_enabled = 0;      // New flag for continuous buzzer control

// For Execution Sequence Check
typedef enum {
    SEQ_STATE_POWER_ON_RESET,       // Initial state after reset
    SEQ_STATE_HAL_INIT_CALLED,      // After HAL_Init() called in init()
    SEQ_STATE_SYSCLK_CONFIG_CALLED, // After SystemClock_Config() called in init()
    SEQ_STATE_MX_PERIPH_INIT_CALLED,// After all MX_..._Init() calls in init()
    SEQ_STATE_CUSTOM_HAL_INIT_CALLED,// After HAL_TIM_PWM_Start, HAL_ADC_Start_DMA in init()
    SEQ_STATE_INIT_FN_COMPLETED,    // init() function is about to return
    SEQ_STATE_MAIN_POST_INIT_FN,    // In main(), immediately after init() returned
    SEQ_STATE_HOT_COLD_CHECK_COMPLETED, // After check_and_restore_hot_start()
    SEQ_STATE_COLD_START_PATH_COMPLETED, // After cold start specific initializations
    SEQ_STATE_HOT_START_PATH_COMPLETED,  // After hot start specific restorations & filter priming
    SEQ_STATE_PRE_MAIN_LOOP,        // All setup before main while(1) loop is done
    SEQ_STATE_IN_MAIN_LOOP          // Consistently inside the main while(1) loop
} ExecutionSequenceState_t;

static volatile ExecutionSequenceState_t g_exec_sequence_state = SEQ_STATE_POWER_ON_RESET;
/* USER CODE END PV */

// Variables for Moving Average Filter
#define FILTER_WINDOW_SIZE 10 // Size of the moving average window (e.g., 10 samples)
static float light_value_history[FILTER_WINDOW_SIZE] = {0.0f}; // History of raw light values
static int filter_index = 0; // Current index in the history buffer
static float current_sum = 0.0f; // Sum of values in the history buffer
static int num_samples_in_filter = 0; // Number of samples currently in the filter


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void display_light_value_on_zlg7290(float value);
void Turn_On_Marquee_LED(uint8_t led_index); // Function to turn on a specific marquee LED
void Turn_Off_All_Marquee_LEDs(void);    // Function to turn off all marquee LEDs
void beer_should_sound(void); // Function to control buzzer sound state
void save_hot_start_state(void); // Function to save hot start state
uint8_t check_and_restore_hot_start(void); // Function to check and restore hot start state
float apply_moving_average_filter(float raw_value); // Function to apply moving average filter
void handle_sequence_error_function(ExecutionSequenceState_t expected, const char* file, int line);
void ProcessKeyPresses(void);
uint8_t ReadZLG7290Key(void); // Function to read key from ZLG7290
/* USER CODE END Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


uint8_t ReadZLG7290Key(void) {
    uint8_t key_buffer[1] = {0}; // Buffer to store the read key value
    I2C_ZLG7290_Read(&hi2c1, 0x71, ZLG_READ_ADDRESS1, key_buffer, 1);
    return key_buffer[0];
}

void ProcessKeyPresses(void) {
    if (g_key_interrupt_flag) {
        g_key_interrupt_flag = 0; // Acknowledge this interrupt instance first

        g_raw_key_value = ReadZLG7290Key();
        // It's important to print the value *after* the debounce and read
        printf("Raw Key Value (debounced): %#x\r\n", g_raw_key_value);

        // Process only if a valid key (not 0 and not I2C error 0xFF) was read after debounce
        if (g_raw_key_value != 0 && g_raw_key_value != 0xFF) {
            switch (g_raw_key_value) {
                case KEY_CODE_1: // Make sure these KEY_CODE_ defines match your actual debounced values
                    g_main_loop_delay_ms = 25;
                    printf("Sample rate set to 500ms delay.\r\n");
                    break;
                case KEY_CODE_2:
                    g_main_loop_delay_ms = 50;
                    printf("Sample rate set to 750ms delay.\r\n");
                    break;
                case KEY_CODE_3:
                    g_main_loop_delay_ms = 75;
                    printf("Sample rate set to 1000ms delay.\r\n");
                    break;
                case KEY_CODE_A: // Example: if 'A' is 0x19 after debounce
                    g_marquee_logic_enabled = 1;
                    printf("Marquee LED logic enabled.\r\n");
                    break;
                case KEY_CODE_B: // Example: if 'B' is 0x11 after debounce
                    g_fan_logic_enabled = 1;
                    printf("Fan logic enabled.\r\n");
                    break;
                case KEY_CODE_C:
                    g_marquee_logic_enabled = 0;
                    g_fan_logic_enabled = 0;
                    Turn_Off_All_Marquee_LEDs(); // Immediately turn off LEDs
                    DC_Task(0x00);               // Immediately stop fan
                    printf("Marquee LED and Fan logic disabled.\r\n");
                    break;
                default:
                    printf("Unknown or unhandled key after debounce: %#x\r\n", g_raw_key_value);
                    // No action for unknown keys
                    break;
            }
        } else if (g_raw_key_value == 0) {
            // This means that after the debounce delay, ZLG7290 reported no key pressed.
            // This can happen if the interrupt was due to noise, or a very brief bounce,
            // or if the key was released during the debounce delay.
            // printf("Debounced read resulted in 0 (no key/end of bounce).\r\n");
        } else { // g_raw_key_value == 0xFF
            printf("I2C Read Error for key after debounce.\r\n");
        }
    }
}


void handle_sequence_error_function(ExecutionSequenceState_t expected, const char* file, int line) {
    printf("FATAL: Execution sequence error! Expected state %d, got %d. File: %s, Line: %d\r\n",
           (int)expected, (int)g_exec_sequence_state, file, line);
    // Optional: Attempt to save some critical state before reset, use with caution
    // save_hot_start_state(); 
    NVIC_SystemReset(); // Re-initialize by system reset
}

#define CHECK_SEQUENCE(expected_state) \
    do { \
        if (g_exec_sequence_state != expected_state) { \
            handle_sequence_error_function(expected_state, __FILE__, __LINE__); \
        } \
    } while(0)
    
float apply_moving_average_filter(float raw_value)
{
    // Subtract the oldest value from sum
    current_sum -= light_value_history[filter_index];
    // Add new raw value to history
    light_value_history[filter_index] = raw_value;
    // Add new raw value to sum
    current_sum += raw_value;
    // Advance index (circularly)
    filter_index = (filter_index + 1) % FILTER_WINDOW_SIZE;

    if (num_samples_in_filter < FILTER_WINDOW_SIZE) {
        num_samples_in_filter++; // Increment sample count until buffer is full
    }

    if (num_samples_in_filter > 0) {
        return current_sum / num_samples_in_filter; // Calculate average
    } else {
        return raw_value; // Should ideally not happen if buffer fills
    }
}

// Data validation function for hot start recovery
uint8_t validate_hot_start_data(void)
{
    // Validate light sensor value range (10-30 based on experiments)
    if (light_value < 5.0f || light_value > 35.0f) {
        printf("Hot start data validation failed: light_value=%.2f out of range\r\n", light_value);
        return 0; // Validation failed
    }
    
    // Validate servo angle range (0-180 degrees)
    if (servoAngle < 0.0f || servoAngle > 180.0f) {
        printf("Hot start data validation failed: servoAngle=%.2f out of range\r\n", servoAngle);
        return 0; // Validation failed
    }
    
    // Validate marquee count range (0-255, but typically 0-3 for LED index)
    if (marquee_count > 100) { // Allow some flexibility
        printf("Hot start data validation failed: marquee_count=%d out of range\r\n", marquee_count);
        return 0; // Validation failed
    }
    
    // Validate buzzer state (should be reasonable) - check g_buzzer_enabled instead
    if (g_buzzer_enabled > 1) { // Should be 0 or 1
        printf("Hot start data validation failed: buzzer_enabled=%d out of range\r\n", g_buzzer_enabled);
        return 0; // Validation failed
    }
    
    // Additional consistency check: servo angle should match light value
    float expected_servo_angle = light_value * (180.0/33.0);
    float angle_difference = (servoAngle > expected_servo_angle) ? 
                            (servoAngle - expected_servo_angle) : 
                            (expected_servo_angle - servoAngle);
    
    if (angle_difference > 10.0f) { // Allow 10 degree tolerance
        printf("Hot start data validation failed: servo angle inconsistent with light value\r\n");
        return 0; // Validation failed
    }
    
    printf("Hot start data validation passed\r\n");
    return 1; // Validation passed
}

// Reset to safe default values when validation fails
void reset_to_safe_defaults(void)
{
    light_value = 15.0f;    // Safe middle value
    servoAngle = 15.0f * (180.0/33.0);
    marquee_count = 0;
    g_buzzer_enabled = 0;   // Changed from g_buzzer_should_sound
    
    printf("Reset to safe default values\r\n");
}

void save_hot_start_state(void)
{
    // Save the current state to backup registers
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    
    // Populate main hot start data
    hot_start_data->hot_start_flag = HOT_START_FLAG_VALUE;
    
    union {
        float f;
        uint32_t u;
    } light_converter;
    light_converter.f = light_value;
    hot_start_data->light_value_raw = light_converter.u;
    
    hot_start_data->marquee_count_saved = marquee_count;
    hot_start_data->buzzer_enabled_saved = g_buzzer_enabled;
    hot_start_data->motor_state_saved = (light_value > 20.0f) ? 1 : 0;
    
    uint32_t checksum = hot_start_data->light_value_raw + 
                       hot_start_data->marquee_count_saved + 
                       hot_start_data->buzzer_enabled_saved + 
                       hot_start_data->motor_state_saved;
    hot_start_data->checksum = ~checksum;

    // Populate validation data (+1 for each field)
    hot_start_validation_data->validation_flag = HOT_START_VALIDATION_FLAG_VALUE;
    hot_start_validation_data->light_value_raw_plus_one = hot_start_data->light_value_raw + 1;
    hot_start_validation_data->marquee_count_saved_plus_one = hot_start_data->marquee_count_saved + 1;
    hot_start_validation_data->buzzer_enabled_saved_plus_one = hot_start_data->buzzer_enabled_saved + 1;
    hot_start_validation_data->motor_state_saved_plus_one = hot_start_data->motor_state_saved + 1;
    
    HAL_PWR_DisableBkUpAccess();
}

uint8_t check_and_restore_hot_start(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    
    // 1. Check main hot start flag
    if (hot_start_data->hot_start_flag != HOT_START_FLAG_VALUE) {
        printf("Hot start main flag invalid or not set.\r\n");
        HAL_PWR_DisableBkUpAccess();
        return 0; // No valid primary hot start data
    }

    // 2. Check validation data flag
    if (hot_start_validation_data->validation_flag != HOT_START_VALIDATION_FLAG_VALUE) {
        printf("Hot start validation flag invalid or not set.\r\n");
        HAL_PWR_DisableBkUpAccess();
        return 0; // No valid validation data
    }
        
    // 3. Verify checksum for main data
    uint32_t calculated_checksum = hot_start_data->light_value_raw + 
                                 hot_start_data->marquee_count_saved + 
                                 hot_start_data->buzzer_enabled_saved + 
                                 hot_start_data->motor_state_saved;
    calculated_checksum = ~calculated_checksum;
    
    if (calculated_checksum != hot_start_data->checksum) {
        printf("Hot start checksum validation failed.\r\n");
        HAL_PWR_DisableBkUpAccess();
        return 0; // Treat as cold start
    }
    
    // 4. Perform the new +1 validation
    if ((hot_start_data->light_value_raw + 1) != hot_start_validation_data->light_value_raw_plus_one) {
        printf("Hot start +1 validation failed for light_value_raw.\r\n");
        HAL_PWR_DisableBkUpAccess();
        return 0;
    }
    if ((hot_start_data->marquee_count_saved + 1) != hot_start_validation_data->marquee_count_saved_plus_one) {
        printf("Hot start +1 validation failed for marquee_count_saved.\r\n");
        HAL_PWR_DisableBkUpAccess();
        return 0;
    }
    if ((hot_start_data->buzzer_enabled_saved + 1) != hot_start_validation_data->buzzer_enabled_saved_plus_one) {
        printf("Hot start +1 validation failed for buzzer_enabled_saved.\r\n");
        HAL_PWR_DisableBkUpAccess();
        return 0;
    }
    if ((hot_start_data->motor_state_saved + 1) != hot_start_validation_data->motor_state_saved_plus_one) {
        printf("Hot start +1 validation failed for motor_state_saved.\r\n");
        HAL_PWR_DisableBkUpAccess();
        return 0;
    }

    // All checks passed, restore the state from main hot start data
    union {
        float f;
        uint32_t u;
    } light_converter;
    light_converter.u = hot_start_data->light_value_raw;
    light_value = light_converter.f;
    
    marquee_count = hot_start_data->marquee_count_saved;
    g_buzzer_enabled = hot_start_data->buzzer_enabled_saved;
    
    // Restore servo angle (derived from light_value)
    servoAngle = light_value * (180.0/33.0); 

    // 5. Validate the restored data semantically (existing function)
    if (!validate_hot_start_data()) {
        // Data validation failed, reset to safe defaults
        reset_to_safe_defaults();
        printf("Hot start data corrupted (semantic validation failed), using safe defaults.\r\n");
        HAL_PWR_DisableBkUpAccess(); // Ensure access is disabled before returning
        return 0; // Treat as cold start due to data corruption
    }
    
    HAL_PWR_DisableBkUpAccess();
    printf("Hot start successful with all validations passed.\r\n");
    return 1; // Hot start with valid data
}

void beer_should_sound(void)
{
    static uint8_t buzzer_pin_state = 0; // Variable to toggle pin state

    if (g_buzzer_enabled) {
        // Toggle the pin state to generate a square wave
        buzzer_pin_state = !buzzer_pin_state;
        if (buzzer_pin_state) {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET); // Set pin high
        } else {
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // Set pin low
        }
    } else {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // Ensure buzzer is off
        buzzer_pin_state = 0; // Reset state when buzzer is disabled to ensure it starts low next time
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

void init() {
  CHECK_SEQUENCE(SEQ_STATE_POWER_ON_RESET); // init() should be called right after power-on/reset
  HAL_Init();
  g_exec_sequence_state = SEQ_STATE_HAL_INIT_CALLED;

  CHECK_SEQUENCE(SEQ_STATE_HAL_INIT_CALLED);
  SystemClock_Config();
  g_exec_sequence_state = SEQ_STATE_SYSCLK_CONFIG_CALLED;

  CHECK_SEQUENCE(SEQ_STATE_SYSCLK_CONFIG_CALLED);
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC3_Init();
  MX_USART1_UART_Init();
  MX_TIM12_Init(); // init PWM timer
  MX_I2C1_Init();  // This is also an MX_ peripheral init
  g_exec_sequence_state = SEQ_STATE_MX_PERIPH_INIT_CALLED;

  /* USER CODE BEGIN 2 in init() - HAL_TIM_PWM_Start and HAL_ADC_Start_DMA are here */  
  CHECK_SEQUENCE(SEQ_STATE_MX_PERIPH_INIT_CALLED);
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); 
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)adcx,4); // Start ADC conversion with DMA
  g_exec_sequence_state = SEQ_STATE_CUSTOM_HAL_INIT_CALLED;
  
  CHECK_SEQUENCE(SEQ_STATE_CUSTOM_HAL_INIT_CALLED); // Final check before init() returns
  g_exec_sequence_state = SEQ_STATE_INIT_FN_COMPLETED;
}

int main(void)
{
  // g_exec_sequence_state is SEQ_STATE_POWER_ON_RESET by default (global static variable)

  /* MCU Configuration----------------------------------------------------------*/
  init(); // This function will internally check and update g_exec_sequence_state
  CHECK_SEQUENCE(SEQ_STATE_INIT_FN_COMPLETED); // Verify init() completed its sequence
  g_exec_sequence_state = SEQ_STATE_MAIN_POST_INIT_FN;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  // Check if it's a hot start
  CHECK_SEQUENCE(SEQ_STATE_MAIN_POST_INIT_FN);
  is_hot_start = check_and_restore_hot_start();
  // check_and_restore_hot_start might also have internal sequence checks if it becomes complex
  g_exec_sequence_state = SEQ_STATE_HOT_COLD_CHECK_COMPLETED;
  Turn_Off_All_Marquee_LEDs();
  DC_Task(0x00);
  g_marquee_logic_enabled = 0; // Explicitly set to off at start
  g_fan_logic_enabled = 0;     // Explicitly set to off at start

  if (!is_hot_start) {
      CHECK_SEQUENCE(SEQ_STATE_HOT_COLD_CHECK_COMPLETED);
      // Cold start: normal initialization
      Turn_Off_All_Marquee_LEDs(); // Ensure marquee LEDs are off at startup
      printf("Cold start detected\r\n");
      for(int i=0; i<FILTER_WINDOW_SIZE; ++i) light_value_history[i] = 0.0f;
      current_sum = 0.0f;
      filter_index = 0;
      num_samples_in_filter = 0;
      temp1 = 0;
      light_value = 0;
      servoAngle = 0.0;
      g_exec_sequence_state = SEQ_STATE_COLD_START_PATH_COMPLETED;
  } else {
      CHECK_SEQUENCE(SEQ_STATE_HOT_COLD_CHECK_COMPLETED);
      // Hot start: Restore device states based on saved state
      if (light_value > 20.0f) {
          Turn_Off_All_Marquee_LEDs();
          Turn_On_Marquee_LED(marquee_count % 4);
          DC_Task(0x13);
      } else {
          Turn_Off_All_Marquee_LEDs();
          DC_Task(0x00);
      }
      SteeringEngine_Rotate(servoAngle);
      // Filter priming for hot start:
      for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
          light_value_history[i] = light_value;
      }
      current_sum = light_value * FILTER_WINDOW_SIZE;
      num_samples_in_filter = FILTER_WINDOW_SIZE;
      filter_index = 0;
      printf("Hot start detected, light_value=%.2f\r\n", light_value);
      g_exec_sequence_state = SEQ_STATE_HOT_START_PATH_COMPLETED;
  }
  /* USER CODE END 2 */
  
  // Check if one of the paths (cold or hot start) was completed
  if (g_exec_sequence_state == SEQ_STATE_COLD_START_PATH_COMPLETED) {
      if (num_samples_in_filter == 0) // This condition is part of cold start path logic
          SteeringEngine_RotateFullCircle(); // Only execute full circle rotation on cold start
  } else if (g_exec_sequence_state == SEQ_STATE_HOT_START_PATH_COMPLETED) {
      // Any specific actions after hot start path completed, if necessary
  } else {
      // Neither path completed, this indicates a flaw in the if/else logic or sequence update
      handle_sequence_error_function(SEQ_STATE_COLD_START_PATH_COMPLETED, __FILE__, __LINE__); // Expected one of the completed path states
  }
  g_exec_sequence_state = SEQ_STATE_PRE_MAIN_LOOP;


  while (1)
  {
    /* USER CODE END WHILE */ // This comment seems misplaced, usually before while(1)
    
    // Check sequence at the beginning of the loop
    if (g_exec_sequence_state == SEQ_STATE_PRE_MAIN_LOOP) { // First iteration
        g_exec_sequence_state = SEQ_STATE_IN_MAIN_LOOP;
    } else {
        CHECK_SEQUENCE(SEQ_STATE_IN_MAIN_LOOP); // Subsequent iterations
    }

    /* USER CODE BEGIN 3 */    
    ProcessKeyPresses(); // Check for and handle key presses

    if (!is_hot_start) { // This 'is_hot_start' check in the loop might need re-evaluation
                         // It's usually cleared after the first iteration post-hot-start.
                         // Assuming it's correctly managed.
        // Normal operation mode: read ADC values
        temp0 = (float)adcx[0]*(3.3/4096); // ADC Channel 0
        temp1 = (float)adcx[1]*(3.3/4096); // ADC Channel 1 (e.g., light sensor)
        temp2 = (float)adcx[2]*(3.3/4096); // ADC Channel 2
        temp3 = (float)adcx[3]*(3.3/4096); // ADC Channel 3
        float raw_light_value_scaled = temp1*10; // Scale light sensor value (example scaling)

        // Apply moving average filter
        light_value = apply_moving_average_filter(raw_light_value_scaled);
        servoAngle = light_value * (180.0/33.0);
    } else {
        // This block is for the first iteration after a hot start.
        // Prime the filter with the restored light_value for a smooth transition.
        for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
            light_value_history[i] = light_value; // Use the restored light_value
        }
        current_sum = light_value * FILTER_WINDOW_SIZE;
        num_samples_in_filter = FILTER_WINDOW_SIZE; // Mark filter as full
        filter_index = 0; // Reset filter index
        // servoAngle is already restored/recalculated in check_and_restore_hot_start or just before this loop.
        // No need to recalculate servoAngle here as light_value hasn't changed yet from restored.

        is_hot_start = 0; // Clear hot start flag, run in normal mode afterwards
    }
    
    printf("\r Light Sensor Value =%f\r",light_value); // Print light sensor value via UART
    printf("\r\n servoAngle = %f\r\n", servoAngle); // Print servo angle
    display_light_value_on_zlg7290(light_value); // Prepare display data for ZLG7290
    // Write data from Tx1_Buffer to ZLG7290 display via I2C
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8);
    SteeringEngine_Rotate(servoAngle); // Rotate steering engine based on light value

    // Marquee LED control
    if (g_marquee_logic_enabled && light_value > 20.0f) {
        Turn_Off_All_Marquee_LEDs();
        Turn_On_Marquee_LED(marquee_count % 4);
        marquee_count++;
    } else {
        Turn_Off_All_Marquee_LEDs(); // Off if logic disabled or light <= 20
    }

    // Fan (DC Motor) control
    if (g_fan_logic_enabled && light_value > 20.0f) {
        DC_Task(0x13); // Rotate DC motor
    } else {
        DC_Task(0x00); // Stop DC motor if logic disabled or light <= 20
    }
    
    // Buzzer control (remains independent of new key flags for now)
    if (light_value > 20.0f) { // Actually a dark environment
        g_buzzer_enabled = 1;
    } else { // Actually a bright environment
        g_buzzer_enabled = 0;
    }
    
    static uint8_t save_counter = 0;
    if (++save_counter >= 5) {
        save_hot_start_state();
        save_counter = 0;
    }

    CHECK_SEQUENCE(SEQ_STATE_IN_MAIN_LOOP);
    HAL_Delay(g_main_loop_delay_ms); // Use variable delay
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

// HAL_GPIO_EXTI_Callback function
// This function will be called when any EXTI line interrupt occurs.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13) // Check for PD13
  {
    g_key_interrupt_flag = 1;
  }
  // If you have other EXTI sources, handle them here too
  // else if (GPIO_Pin == OTHER_PIN) { ... }
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