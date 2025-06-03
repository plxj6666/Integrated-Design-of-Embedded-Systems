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
#include "stm32f4xx_hal_iwdg.h"
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
    uint32_t light_value_raw;          // Raw light value (float converted to uint32_t)
    uint32_t packed_states;            // Holds marquee_count, buzzer_enabled, motor_state, main_loop_delay
    uint32_t checksum;                 // Checksum for hot_start_flag, light_value_raw, packed_states
} __attribute__((packed)) __attribute__((aligned(4))) HotStartData_t; // Size: 4 * sizeof(uint32_t) = 16 bytes

typedef struct {
    uint32_t validation_flag;           // Should be HOT_START_VALIDATION_FLAG_VALUE
    uint32_t light_value_raw_plus_one;
    uint32_t packed_states_plus_one;    // Packed data with each original field incremented by 1
    uint32_t checksum;                  // Checksum for validation_flag, light_value_raw_plus_one, packed_states_plus_one
} __attribute__((packed)) __attribute__((aligned(4))) HotStartValidationData_t; // Size: 4 * sizeof(uint32_t) = 16 bytes

// STM32F4备份寄存器的正确基地址
#define BACKUP_REG_BASE_ADDR ((uint32_t *)0x40002850)  // 如果这是RTC备份寄存器

#define hot_start_data ((volatile HotStartData_t *)BACKUP_REG_BASE_ADDR)
// sizeof(HotStartData_t) is 16 bytes (4 uint32_t words)
// Offsets are in terms of uint32_t* pointer arithmetic
#define hot_start_data ((volatile HotStartData_t *)BACKUP_REG_BASE_ADDR)
#define hot_start_validation_data_1 ((volatile HotStartValidationData_t *)(BACKUP_REG_BASE_ADDR + 4))
#define hot_start_validation_data_2 ((volatile HotStartValidationData_t *)(BACKUP_REG_BASE_ADDR + 8))
#define hot_start_validation_data_3 ((volatile HotStartValidationData_t *)(BACKUP_REG_BASE_ADDR + 12))

uint8_t is_hot_start = 0;  // hot start flag
static uint8_t first_loop_after_hot_start = 0; // 移到全局变量区域

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
uint8_t g_loop_check = 0;
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

#define SHOULE_INIT 100
uint8_t marquee_count = 0; // Counter for marquee LED sequence
uint8_t gear = 2; // Current gear state (1-3, 0 = off)
uint8_t init_num = 0;
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
IWDG_HandleTypeDef hiwdg;
static uint32_t g_watchdog_feed_counter = 0;
#define WATCHDOG_FEED_INTERVAL 100  // 每30次主循环喂一次狗

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
// void save(void); // Function to save hot start state // Assuming save_state is the correct name
void save_state(void); // Corrected function name
// uint8_t check_and_restore_hot_start(void); // Function to check and restore hot start state // Original function, will be replaced
uint8_t check_initial_hot_start_validity(void); // New function: Part 1 of hot start check
uint8_t validate_all_data_and_restore_state(uint8_t main_data_initially_valid); // New function: Part 2 of hot start check & restore
float apply_moving_average_filter(float raw_value); // Function to apply moving average filter
void handle_sequence_error_function(ExecutionSequenceState_t expected, const char* file, int line);
void ProcessKeyPresses(void);
uint8_t ReadZLG7290Key(void); // Function to read key from ZLG7290

void MX_IWDG_Init(void);
void Watchdog_Feed(void);
void Error_Handler(void);
void Sleep(void); // Function to put ADC into sleep mode
void Wakeup(void); // Function to wake up ADC from sleep mode
/* USER CODE END Private function prototypes -----------------------------------------------*/
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// 添加ADC休眠控制函数
void Sleep(void)
{
    // 停止ADC DMA转换
    HAL_ADC_Stop_DMA(&hadc3);
    
    // 关闭ADC外设
    HAL_ADC_DeInit(&hadc3);
    
    // 可选：关闭ADC时钟以进一步节省功耗
    __HAL_RCC_ADC3_CLK_DISABLE();
    
    printf("ADC entered sleep mode\r\n");
}

void Wakeup(void)
{
    // 重新使能ADC时钟
    __HAL_RCC_ADC3_CLK_ENABLE();
    
    // 重新初始化ADC
    MX_ADC3_Init();
    
    // 重新启动ADC DMA转换
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adcx, 4);
    
    printf("ADC woke up from sleep mode\r\n");
}

void MX_IWDG_Init(void)
{
  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;    // 分频系数64
  hiwdg.Init.Reload = 625;                     // 重载值625
  // 超时时间 = (4 * Prescaler * Reload) / LSI_Freq
  // LSI大约32kHz，所以超时时间 ≈ (256 * 64 * 625) / 32000 ≈ 20秒
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
  printf("Watchdog initialized with 5 second timeout\r\n");
  /* USER CODE END IWDG_Init 2 */
}

/**
  * @brief Feed the watchdog
  * @param None
  * @retval None
  */
void Watchdog_Feed(void)
{
  HAL_IWDG_Refresh(&hiwdg);
  // printf("Watchdog fed\r\n");  // 可选：打印喂狗信息，但会增加串口输出
}

// Error handler function
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  printf("Error: System error occurred, entering infinite loop\r\n");
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

uint8_t ReadZLG7290Key(void) {
    uint8_t key_buffer[1] = {0}; // Buffer to store the read key value
    // Consider adding a small delay here if I2C bus is shared and might be busy,
    // or if ZLG7290 needs a moment between reads, though usually not necessary for simple reads.
    // HAL_Delay(1); // Example: 1ms delay, if needed.
    I2C_ZLG7290_Read(&hi2c1, 0x71, ZLG_READ_ADDRESS1, key_buffer, 1);
    return key_buffer[0];
}

void ProcessKeyPresses(void) {
    if (g_key_interrupt_flag) {
        g_key_interrupt_flag = 0; // Acknowledge this interrupt instance first

        uint8_t key1, key2, key3;

        key1 = ReadZLG7290Key(); // First read
        key2 = ReadZLG7290Key(); // Second read

        if (key1 == key2) {
            g_raw_key_value = key1; // Values match, consider it stable
        } else {
            // Values don't match, try a third read
            key3 = ReadZLG7290Key();
            g_raw_key_value = key3; // Use the third read as the definitive value
            // Optionally, you could add more sophisticated logic here,
            // e.g., if key3 matches key1 or key2, or a voting system if more reads are done.
            // For now, we just take the third read.
            printf("Key values mismatched (k1=%#x, k2=%#x), used third read (k3=%#x).\r\n", key1, key2, key3);
        }

        // It's important to print the value *after* the confirmation logic
        printf("Confirmed Key Value: %#x\r\n", g_raw_key_value);

        // Process only if a valid key (not 0 and not I2C error 0xFF) was read after confirmation
        if (g_raw_key_value != 0 && g_raw_key_value != 0xFF) {
            switch (g_raw_key_value) {
                case KEY_CODE_1: // Make sure these KEY_CODE_ defines match your actual debounced values
                    g_main_loop_delay_ms = 25;
                    printf("Sample rate set to 25ms base delay.\r\n"); // Corrected printf
                    break;
                case KEY_CODE_2:
                    g_main_loop_delay_ms = 50;
                    printf("Sample rate set to 50ms base delay.\r\n"); // Corrected printf
                    break;
                case KEY_CODE_3:
                    g_main_loop_delay_ms = 75;
                    printf("Sample rate set to 75ms base delay.\r\n"); // Corrected printf
                    break;
                case KEY_CODE_A: // Example: if 'A' is 0x19 after debounce
                    g_marquee_logic_enabled = !g_marquee_logic_enabled; // Toggle
                    if (g_marquee_logic_enabled) {
                        printf("Marquee LED logic enabled.\r\n");
                    } else {
                        printf("Marquee LED logic disabled.\r\n");
                        Turn_Off_All_Marquee_LEDs(); // Turn off LEDs when logic is disabled
                    }
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
            // This means that after the confirmation reads, ZLG7290 reported no key pressed.
            // This can happen if the key was released very quickly or due to noise.
            // printf("Confirmed read resulted in 0 (no key/end of bounce).\r\n");
        } else { // g_raw_key_value == 0xFF
            printf("I2C Read Error for key after confirmation.\r\n");
        }
    }
}


void handle_sequence_error_function(ExecutionSequenceState_t expected, const char* file, int line) {
    printf("FATAL: Execution sequence error! Expected state %d, got %d. File: %s, Line: %d\r\n",
           (int)expected, (int)g_exec_sequence_state, file, line);
    // Optional: Attempt to save some critical state before reset, use with caution
    // save_state(); 
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

// Reset to safe default values when validation fails
void reset_to_safe_defaults(void)
{
    light_value = 15.0f;    // Safe middle value
    servoAngle = 15.0f * (180.0/33.0);
    marquee_count = 0;
    g_buzzer_enabled = 0;   // Changed from g_buzzer_should_sound
    g_main_loop_delay_ms = 50;  // 设置为默认档位2
    
    printf("Reset to safe default values (delay: %lu ms)\r\n", g_main_loop_delay_ms);
}

void save_state(void)
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
    
    uint8_t mc_save = marquee_count;
    uint8_t be_save = g_buzzer_enabled ? 1 : 0;
    uint8_t ms_save = (light_value > 20.0f) ? 1 : 0; // Example motor state logic
    uint8_t mld_save = (uint8_t)g_main_loop_delay_ms;

    hot_start_data->packed_states = (mc_save & 0xFFU) |          // Bits 0-7
                                    ((be_save & 0x01U) << 8) |   // Bit 8
                                    ((ms_save & 0x01U) << 9) |   // Bit 9
                                    ((mld_save & 0xFFU) << 10);  // Bits 10-17
    
    uint32_t checksum_payload = hot_start_data->hot_start_flag + 
                                hot_start_data->light_value_raw + 
                                hot_start_data->packed_states;
    hot_start_data->checksum = ~checksum_payload;

    // Populate validation data (+1 for each field, then pack)
    uint16_t mc_save_p1 = mc_save + 1;
    uint8_t be_save_p1 = be_save + 1;
    uint8_t ms_save_p1 = ms_save + 1;
    uint16_t mld_save_p1 = mld_save + 1;

    uint32_t packed_plus_one_value = (mc_save_p1 & 0x1FFU) |         // Bits 0-8
                                     ((be_save_p1 & 0x03U) << 9) |   // Bits 9-10
                                     ((ms_save_p1 & 0x03U) << 11) |  // Bits 11-12
                                     ((mld_save_p1 & 0x1FFU) << 13); // Bits 13-21

    // Validation Set 1
    hot_start_validation_data_1->validation_flag = HOT_START_VALIDATION_FLAG_VALUE;
    hot_start_validation_data_1->light_value_raw_plus_one = hot_start_data->light_value_raw + 1;
    hot_start_validation_data_1->packed_states_plus_one = packed_plus_one_value;
    uint32_t checksum_payload_val1 = hot_start_validation_data_1->validation_flag +
                                     hot_start_validation_data_1->light_value_raw_plus_one +
                                     hot_start_validation_data_1->packed_states_plus_one;
    hot_start_validation_data_1->checksum = ~checksum_payload_val1;

    // Validation Set 2
    hot_start_validation_data_2->validation_flag = HOT_START_VALIDATION_FLAG_VALUE;
    hot_start_validation_data_2->light_value_raw_plus_one = hot_start_data->light_value_raw + 1;
    hot_start_validation_data_2->packed_states_plus_one = packed_plus_one_value;
    uint32_t checksum_payload_val2 = hot_start_validation_data_2->validation_flag +
                                     hot_start_validation_data_2->light_value_raw_plus_one +
                                     hot_start_validation_data_2->packed_states_plus_one;
    hot_start_validation_data_2->checksum = ~checksum_payload_val2;
    
    // Validation Set 3
    hot_start_validation_data_3->validation_flag = HOT_START_VALIDATION_FLAG_VALUE;
    hot_start_validation_data_3->light_value_raw_plus_one = hot_start_data->light_value_raw + 1;
    hot_start_validation_data_3->packed_states_plus_one = packed_plus_one_value;
    uint32_t checksum_payload_val3 = hot_start_validation_data_3->validation_flag +
                                     hot_start_validation_data_3->light_value_raw_plus_one +
                                     hot_start_validation_data_3->packed_states_plus_one;
    hot_start_validation_data_3->checksum = ~checksum_payload_val3;

    HAL_PWR_DisableBkUpAccess();
}

uint8_t periodically_check_and_repair_backup_integrity(void)
{
    uint8_t repair_action_taken = 0; // 0: no repair, 1: main repaired, 2: backup repaired, 3: both

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    // 1. Check Main Data Integrity
    uint8_t main_data_structurally_valid = 0;
    if (hot_start_data->hot_start_flag == HOT_START_FLAG_VALUE) {
        uint32_t expected_main_checksum_payload = hot_start_data->hot_start_flag +
                                                  hot_start_data->light_value_raw +
                                                  hot_start_data->packed_states;
        if (hot_start_data->checksum == ~expected_main_checksum_payload) {
            main_data_structurally_valid = 1;
        }
    }

    // 2. Check Backup Data Integrity (Initial Pass)
    uint8_t backup_initially_valid[3] = {0, 0, 0};
    int first_valid_backup_idx = -1;
    uint8_t initial_valid_backup_count = 0;

    volatile HotStartValidationData_t* validation_sets[] = {
        hot_start_validation_data_1,
        hot_start_validation_data_2,
        hot_start_validation_data_3
    };
    for (int i = 0; i < 3; ++i) {
        if (validation_sets[i]->validation_flag == HOT_START_VALIDATION_FLAG_VALUE) {
            uint32_t expected_val_checksum = validation_sets[i]->validation_flag +
                                             validation_sets[i]->light_value_raw_plus_one +
                                             validation_sets[i]->packed_states_plus_one;
            if (validation_sets[i]->checksum == ~expected_val_checksum) {
                backup_initially_valid[i] = 1;
                initial_valid_backup_count++;
                if (first_valid_backup_idx == -1) {
                    first_valid_backup_idx = i;
                }
            }
        }
    }
    // 3. Attempt to Repair Main Data (if needed and possible)
    if (!main_data_structurally_valid && first_valid_backup_idx != -1) {
        printf("Periodic Check: Main data corrupted. Attempting repair from backup %d.\r\n", first_valid_backup_idx + 1);
        volatile HotStartValidationData_t* repair_source = validation_sets[first_valid_backup_idx];

        hot_start_data->hot_start_flag = HOT_START_FLAG_VALUE; // Flag is known from backup's existence
        hot_start_data->light_value_raw = repair_source->light_value_raw_plus_one - 1;

        uint32_t packed_plus_one_val = repair_source->packed_states_plus_one;
        uint16_t mc_p1 = (packed_plus_one_val) & 0x1FFU;
        uint8_t  be_p1 = (packed_plus_one_val >> 9) & 0x03U;
        uint8_t  ms_p1 = (packed_plus_one_val >> 11) & 0x03U;
        uint16_t mld_p1= (packed_plus_one_val >> 13) & 0x1FFU;

        hot_start_data->packed_states = (((mc_p1 > 0) ? mc_p1 - 1 : 0) & 0xFFU) |
                                        ((((be_p1 > 0) ? be_p1 - 1 : 0) & 0x01U) << 8) |
                                        ((((ms_p1 > 0) ? ms_p1 - 1 : 0) & 0x01U) << 9) |
                                        ((((mld_p1 > 0) ? mld_p1 - 1 : 0) & 0xFFU) << 10);
        
        uint32_t new_main_checksum_payload = hot_start_data->hot_start_flag +
                                             hot_start_data->light_value_raw +
                                             hot_start_data->packed_states;
        hot_start_data->checksum = ~new_main_checksum_payload;

        // Re-verify main data
        if (hot_start_data->checksum == ~new_main_checksum_payload) {
            main_data_structurally_valid = 1;
            repair_action_taken |= 0x01; // Bit 0 for main data repair
            printf("Periodic Check: Main data repair successful.\r\n");
        } else {
            printf("Periodic Check: Main data repair FAILED verification.\r\n");
        }
    }
    // 4. Attempt to Repair Backup Data (if main data is now valid)
    if (main_data_structurally_valid) {
        uint8_t backup_repair_attempted = 0;
        for (int i = 0; i < 3; ++i) {
            if (!backup_initially_valid[i]) { // If this backup was initially invalid
                backup_repair_attempted = 1;
                printf("Periodic Check: Backup set %d corrupted. Attempting repair from main data.\r\n", i + 1);
                volatile HotStartValidationData_t* target_backup = validation_sets[i];

                target_backup->validation_flag = HOT_START_VALIDATION_FLAG_VALUE;
                target_backup->light_value_raw_plus_one = hot_start_data->light_value_raw + 1;

                uint8_t mc_main  = (uint8_t)(hot_start_data->packed_states & 0xFFU);
                uint8_t be_main  = (uint8_t)((hot_start_data->packed_states >> 8) & 0x01U);
                uint8_t ms_main  = (uint8_t)((hot_start_data->packed_states >> 9) & 0x01U);
                uint8_t mld_main = (uint8_t)((hot_start_data->packed_states >> 10) & 0xFFU);

                target_backup->packed_states_plus_one = ((mc_main + 1) & 0x1FFU) |
                                                        (((be_main + 1) & 0x03U) << 9) |
                                                        (((ms_main + 1) & 0x03U) << 11) |
                                                        (((mld_main + 1) & 0x1FFU) << 13);
                
                uint32_t repaired_backup_payload = target_backup->validation_flag +
                                                   target_backup->light_value_raw_plus_one +
                                                   target_backup->packed_states_plus_one;
                target_backup->checksum = ~repaired_backup_payload;
                
                if (target_backup->checksum == ~repaired_backup_payload) {
                     printf("Periodic Check: Backup set %d repair successful.\r\n", i + 1);
                } else {
                     printf("Periodic Check: Backup set %d repair FAILED verification.\r\n", i + 1);
                }
            }
        }
        if (backup_repair_attempted) { // If any backup repair was attempted, set the flag
            repair_action_taken |= 0x02; // Bit 1 for backup data repair
        }
    }

    HAL_PWR_DisableBkUpAccess();
    return repair_action_taken;
}

/**
  * @brief  Checks if a hot start attempt is indicated by the primary flag.
  * @retval 1 if hot_start_data->hot_start_flag is correct, 0 otherwise.
  */
uint8_t check_initial_hot_start_validity(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    uint8_t is_flag_set_correctly = 0;
    if (hot_start_data->hot_start_flag == HOT_START_FLAG_VALUE) {
        is_flag_set_correctly = 1;
        printf("Initial check: Hot start flag is SET correctly.\r\n");
    } else {
        printf("Initial check: Hot start flag is NOT SET or incorrect (value: 0x%08lX).\r\n", hot_start_data->hot_start_flag);
    }
    
    HAL_PWR_DisableBkUpAccess(); 
    return is_flag_set_correctly;
}

/**
  * @brief  Validates main and backup hot start data, attempts repair if needed, and restores state.
  *         Repairs main data from a valid backup.
  *         Repairs invalid backups using valid main data (if main data becomes valid).
  * @param  is_potential_hot_start: Result from check_initial_hot_start_validity().
  * @retval 1 if data is successfully validated/repaired and restored for hot start, 0 for cold start.
  */
uint8_t validate_all_data_and_restore_state(uint8_t is_potential_hot_start)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (!is_potential_hot_start) {
        printf("Validation: Not a potential hot start based on initial flag check. Forcing cold start.\r\n");
        HAL_PWR_DisableBkUpAccess();
        return 0; // Not even an attempt at hot start
    }

    // At this point, hot_start_data->hot_start_flag is HOT_START_FLAG_VALUE.
    // Now, validate main data checksum.
    uint8_t main_data_checksum_valid = 0;
    uint32_t expected_main_checksum_payload = hot_start_data->hot_start_flag +
                                              hot_start_data->light_value_raw +
                                              hot_start_data->packed_states;
    if (hot_start_data->checksum == ~expected_main_checksum_payload) {
        main_data_checksum_valid = 1;
        printf("Validation: Main data checksum initially VALID.\r\n");
    } else {
        printf("Validation: Main data checksum initially INVALID.\r\n");
    }

    // Validate backup sets
    uint8_t backup_is_structurally_valid[3] = {0, 0, 0}; // Flag and checksum OK
    int first_valid_backup_idx = -1;
    uint8_t valid_backup_count = 0;

    volatile HotStartValidationData_t* validation_sets[] = {
        hot_start_validation_data_1,
        hot_start_validation_data_2,
        hot_start_validation_data_3
    };

    printf("--- Backup Set Initial Validation Pass ---\r\n");
    for (int i = 0; i < 3; ++i) {
        uint32_t expected_val_checksum_payload = validation_sets[i]->validation_flag +
                                                 validation_sets[i]->light_value_raw_plus_one +
                                                 validation_sets[i]->packed_states_plus_one;
        if (validation_sets[i]->validation_flag == HOT_START_VALIDATION_FLAG_VALUE &&
            validation_sets[i]->checksum == ~expected_val_checksum_payload) {
            backup_is_structurally_valid[i] = 1;
            valid_backup_count++;
            if (first_valid_backup_idx == -1) {
                first_valid_backup_idx = i;
            }
            printf("Backup set %d: Initially structurally VALID.\r\n", i + 1);
        } else {
            printf("Backup set %d: Initially structurally INVALID (flag/checksum error).\r\n", i + 1);
        }
    }
    printf("Initial valid backup sets: %d/3.\r\n", valid_backup_count);

    // Attempt to repair Main Data if its checksum was invalid AND a valid backup exists
    if (!main_data_checksum_valid && first_valid_backup_idx != -1) {
        printf("Main data checksum invalid. Attempting repair using backup set %d...\r\n", first_valid_backup_idx + 1);
        volatile HotStartValidationData_t* repair_source_for_main = validation_sets[first_valid_backup_idx];
        
        // hot_start_data->hot_start_flag is already known to be correct.
        hot_start_data->light_value_raw = repair_source_for_main->light_value_raw_plus_one - 1;
        
        uint32_t packed_plus_one_from_backup = repair_source_for_main->packed_states_plus_one;
        uint16_t mc_p1_b = (packed_plus_one_from_backup) & 0x1FFU;
        uint8_t  be_p1_b = (packed_plus_one_from_backup >> 9) & 0x03U;
        uint8_t  ms_p1_b = (packed_plus_one_from_backup >> 11) & 0x03U;
        uint16_t mld_p1_b= (packed_plus_one_from_backup >> 13) & 0x1FFU;
        
        hot_start_data->packed_states = (((mc_p1_b > 0) ? mc_p1_b - 1 : 0) & 0xFFU) |
                                        ((((be_p1_b > 0) ? be_p1_b - 1 : 0) & 0x01U) << 8) |
                                        ((((ms_p1_b > 0) ? ms_p1_b - 1 : 0) & 0x01U) << 9) |
                                        ((((mld_p1_b > 0) ? mld_p1_b - 1 : 0) & 0xFFU) << 10);
        
        uint32_t new_main_checksum_payload = hot_start_data->hot_start_flag +
                                             hot_start_data->light_value_raw +
                                             hot_start_data->packed_states;
        hot_start_data->checksum = ~new_main_checksum_payload;
        
        // Re-validate main data checksum after repair
        if (hot_start_data->checksum == ~new_main_checksum_payload) {
            main_data_checksum_valid = 1;
            printf("Main data repaired successfully using backup set %d.\r\n", first_valid_backup_idx + 1);
        } else {
            printf("ERROR: Main data repair FAILED verification. Main data still considered invalid.\r\n");
        }
    } else if (!main_data_checksum_valid) {
        printf("Main data checksum invalid, and no valid backup found to repair it.\r\n");
    }

    // If Main Data is now valid (flag is correct and checksum is valid), attempt to repair any invalid Backup Sets
    if (hot_start_data->hot_start_flag == HOT_START_FLAG_VALUE && main_data_checksum_valid) {
        printf("--- Backup Set Repair Pass (using current main data as source) ---\r\n");
        for (int i = 0; i < 3; ++i) {
            if (backup_is_structurally_valid[i] == 0) { // If this backup was initially (and still) invalid
                printf("Attempting to repair backup set %d using current main data...\r\n", i + 1);
                volatile HotStartValidationData_t* target_backup_to_repair = validation_sets[i];

                target_backup_to_repair->validation_flag = HOT_START_VALIDATION_FLAG_VALUE;
                target_backup_to_repair->light_value_raw_plus_one = hot_start_data->light_value_raw + 1;

                uint8_t mc_main  = (uint8_t)(hot_start_data->packed_states & 0xFFU);
                uint8_t be_main  = (uint8_t)((hot_start_data->packed_states >> 8) & 0x01U);
                uint8_t ms_main  = (uint8_t)((hot_start_data->packed_states >> 9) & 0x01U);
                uint8_t mld_main = (uint8_t)((hot_start_data->packed_states >> 10) & 0xFFU);

                target_backup_to_repair->packed_states_plus_one = ((mc_main + 1) & 0x1FFU) |
                                                                  (((be_main + 1) & 0x03U) << 9) |
                                                                  (((ms_main + 1) & 0x03U) << 11) |
                                                                  (((mld_main + 1) & 0x1FFU) << 13);
                
                uint32_t repaired_backup_checksum_payload = target_backup_to_repair->validation_flag +
                                                            target_backup_to_repair->light_value_raw_plus_one +
                                                            target_backup_to_repair->packed_states_plus_one;
                target_backup_to_repair->checksum = ~repaired_backup_checksum_payload;
                
                // Verify if repair was successful
                if (target_backup_to_repair->validation_flag == HOT_START_VALIDATION_FLAG_VALUE &&
                    target_backup_to_repair->checksum == ~repaired_backup_checksum_payload) {
                    printf("Backup set %d repaired successfully.\r\n", i + 1);
                    // backup_is_structurally_valid[i] = 1; // Update status for final count
                } else {
                    printf("ERROR: Backup set %d repair FAILED verification.\r\n");
                }
            }
        }
        // Recalculate valid_backup_count after repair attempts
        valid_backup_count = 0;
        for(int i=0; i<3; ++i) {
            uint32_t expected_val_check_payload_after_repair = validation_sets[i]->validation_flag +
                                                     validation_sets[i]->light_value_raw_plus_one +
                                                     validation_sets[i]->packed_states_plus_one;
            if (validation_sets[i]->validation_flag == HOT_START_VALIDATION_FLAG_VALUE &&
                validation_sets[i]->checksum == ~expected_val_check_payload_after_repair) {
                valid_backup_count++;
            }
        }
        printf("Total valid backup sets after repair attempts: %d/3.\r\n", valid_backup_count);
    }


    // Final Decision: Hot start only if main data flag is correct AND main data checksum is valid.
    if (hot_start_data->hot_start_flag == HOT_START_FLAG_VALUE && main_data_checksum_valid) {
        printf("Hot start SUCCESS: Main data is valid (Flag: OK, Checksum: OK). %d/3 backups are valid.\r\n", valid_backup_count);
        
        // Restore global variables from hot_start_data
        union {
            float f;
            uint32_t u;
        } light_converter_restore;
        light_converter_restore.u = hot_start_data->light_value_raw;
        light_value = light_converter_restore.f;
        
        uint32_t temp_packed_states = hot_start_data->packed_states;
        marquee_count = (uint8_t)(temp_packed_states & 0xFFU);
        g_buzzer_enabled = (uint8_t)((temp_packed_states >> 8) & 0x01U);
        // motor state was bit 9, not directly restored to a global var in original code, assumed handled by light_value
        g_main_loop_delay_ms = (uint32_t)((temp_packed_states >> 10) & 0xFFU);
        if (g_main_loop_delay_ms == 0) g_main_loop_delay_ms = 50; // Ensure a valid delay
        
        servoAngle = light_value * (180.0f/33.0f); 
    }
    else {
        printf("Hot start FAILED: Main data is invalid (Flag: %s, Checksum: %s). Defaulting to cold start.\r\n",
               (hot_start_data->hot_start_flag == HOT_START_FLAG_VALUE) ? "OK" : "NOT OK",
               (main_data_checksum_valid) ? "OK" : "NOT OK");
        
        // Reset to safe defaults
        reset_to_safe_defaults();
        return 0; // Cold start
    }

    HAL_PWR_DisableBkUpAccess();
    printf("Hot start successful. Restored delay: %lu ms, Light: %.2f\r\n", g_main_loop_delay_ms, light_value);
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
    
    if (g_main_loop_delay_ms == 25) {
        gear = 1;
    } else if (g_main_loop_delay_ms == 50) {
        gear = 2;
    } else if (g_main_loop_delay_ms == 75) {
        gear = 3;
    } else {
        gear = 0; // Default to off if no valid delay is set
    }
    Tx1_Buffer[0] = segment_codes[gear]; // Display gear on DIG1 (leftmost digit)
    // Handle leading zero for values less than 10 (e.g., 05.23)
    if (digit1 == 0 && value < 10.0f) {
        Tx1_Buffer[2] = SEG_BLANK; // Or segment_codes[0] to display leading '0'
    } else {
        Tx1_Buffer[2] = segment_codes[digit1];
    }
    
    Tx1_Buffer[3] = segment_codes[digit2] | SEG_DP; // Ones digit with decimal point
    Tx1_Buffer[4] = segment_codes[digit3];          // Tenths digit
    Tx1_Buffer[5] = segment_codes[digit4];          // Hundredths digit
    
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
//   HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); 
  HAL_ADC_Start_DMA(&hadc3,(uint32_t*)adcx,4); // Start ADC conversion with DMA
  MX_IWDG_Init();  // 添加看门狗初始化
  g_exec_sequence_state = SEQ_STATE_CUSTOM_HAL_INIT_CALLED;
  
  CHECK_SEQUENCE(SEQ_STATE_CUSTOM_HAL_INIT_CALLED); // Final check before init() returns
  g_exec_sequence_state = SEQ_STATE_INIT_FN_COMPLETED;
}


void execute_display_control(void)
{
    display_light_value_on_zlg7290(light_value);
    I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8);
}

// 舵机控制函数
void execute_servo_control(void)
{
    if (first_loop_after_hot_start) {
        // 热启动第一次循环，跳过舵机控制，PWM值已经在启动时通过以上逻辑正确设置
        printf("Hot start: Skipping servo rotation in first loop\r\n");
        first_loop_after_hot_start = 0; // 清除标志，后续循环正常控制舵机
    } else {
        // 正常情况下调用SteeringEngine_Rotate
        SteeringEngine_Rotate(servoAngle);
    }
}

// 跑马灯控制函数
void execute_marquee_control(void)
{
    if (g_marquee_logic_enabled && light_value > 20.0f) {
        Turn_Off_All_Marquee_LEDs();
        Turn_On_Marquee_LED(marquee_count % 4);
        marquee_count++;
    } else {
        Turn_Off_All_Marquee_LEDs();
    }
}

// 风扇控制函数
void execute_fan_control(void)
{
    if (g_fan_logic_enabled && light_value > 20.0f) {
        DC_Task(0x13);
    } else {
        DC_Task(0x00);
    }
}

// 蜂鸣器控制函数
void execute_buzzer_control(void)
{
    if (light_value > 20.0f) {
        g_buzzer_enabled = 1;
    } else {
        g_buzzer_enabled = 0;
    }
}

// 定义函数指针类型
typedef void (*control_function_t)(void);

// 函数指针数组
static control_function_t control_functions[5] = {
    execute_display_control,
    execute_servo_control,
    execute_marquee_control,
    execute_fan_control,
    execute_buzzer_control
};

// 简单的线性同余随机数生成器
static uint32_t rng_state = 12345; // 初始种子

uint32_t simple_random(void)
{
    rng_state = (rng_state * 1103515245 + 12345) & 0x7FFFFFFF;
    return rng_state;
}

// Fisher-Yates洗牌算法来随机化执行顺序
void randomize_execution_order(void)
{
    uint8_t indices[5] = {0, 1, 2, 3, 4};
    
    // Fisher-Yates洗牌
    for (int i = 4; i > 0; i--) {
        uint32_t j = simple_random() % (i + 1);
        // 交换indices[i]和indices[j]
        uint8_t temp = indices[i];
        indices[i] = indices[j];
        indices[j] = temp;
    }
    
    // 按随机顺序执行函数
    for (int i = 0; i < 5; i++) {
        control_functions[indices[i]]();
    }
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

  uint8_t is_potential_hot_start = check_initial_hot_start_validity();
  if (validate_all_data_and_restore_state(is_potential_hot_start)) {
      is_hot_start = 1;
  } else {
      is_hot_start = 0;
      // reset_to_safe_defaults(); // Optionally call this if validation fails to ensure clean state for cold start
      // This printf is now inside validate_all_data_and_restore_state if it fails.
      // printf("Proceeding with cold start due to failed hot start validation/restore.\r\n");
  }
  
  g_exec_sequence_state = SEQ_STATE_HOT_COLD_CHECK_COMPLETED; // This state is after attempting hot/cold determination
  
  Turn_Off_All_Marquee_LEDs();
  DC_Task(0x00);
  // These might be overwritten by restored values if hot start was successful and they are part of packed_states
  // However, g_marquee_logic_enabled and g_fan_logic_enabled are NOT currently in packed_states.
  // They are controlled by key presses.
  g_marquee_logic_enabled = 0; 
  g_fan_logic_enabled = 0;   

  if (!is_hot_start) {
      HAL_Delay(100); 
      CHECK_SEQUENCE(SEQ_STATE_HOT_COLD_CHECK_COMPLETED); 
      printf("Cold start detected or forced by validation failure.\r\n");
      
      servoAngle = 0.0; 
      uint16_t pulse = (uint16_t)(500 + servoAngle * (2000/180.0));
      __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pulse);
      
      htim12.Instance->EGR = TIM_EGR_UG; 
      while((htim12.Instance->SR & TIM_SR_UIF) == 0) {}
      __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE); 

      HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
      
      for(int i=0; i<FILTER_WINDOW_SIZE; ++i) light_value_history[i] = 0.0f;
      current_sum = 0.0f;
      filter_index = 0;
      num_samples_in_filter = 0;
      temp1 = 0; // Assuming temp1 is a global or static float for ADC result
      light_value = 0.0f; 
      g_main_loop_delay_ms = 50; 
      marquee_count = 0;         
      g_buzzer_enabled = 0;      

      g_exec_sequence_state = SEQ_STATE_COLD_START_PATH_COMPLETED;
      first_loop_after_hot_start = 0;
  } else { // This is the hot start path
      CHECK_SEQUENCE(SEQ_STATE_HOT_COLD_CHECK_COMPLETED); 
      // servoAngle, light_value, g_main_loop_delay_ms etc. are already restored.
      
      uint16_t pulse = (uint16_t)(500 + servoAngle * (2000/180.0));
      __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pulse);

      htim12.Instance->EGR = TIM_EGR_UG; 
      while((htim12.Instance->SR & TIM_SR_UIF) == 0) {}
      __HAL_TIM_CLEAR_FLAG(&htim12, TIM_FLAG_UPDATE); 

      HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
      
      if (light_value > 20.0f) {
          // marquee_count is restored. g_marquee_logic_enabled is by key press.
          // If g_marquee_logic_enabled was also part of packed_states and restored, this would be automatic.
          // For now, marquee will only run if g_marquee_logic_enabled is later set by key.
          DC_Task(0x13);
      } else {
          DC_Task(0x00);
      }
      
      for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
          light_value_history[i] = light_value;
      }
      current_sum = light_value * FILTER_WINDOW_SIZE;
      num_samples_in_filter = FILTER_WINDOW_SIZE;
      filter_index = 0;
      g_exec_sequence_state = SEQ_STATE_HOT_START_PATH_COMPLETED;
      first_loop_after_hot_start = 1; 
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
    // Check sequence at the beginning of the loop
    if (g_exec_sequence_state == SEQ_STATE_PRE_MAIN_LOOP) { // First iteration
        g_exec_sequence_state = SEQ_STATE_IN_MAIN_LOOP;
    } else {
        CHECK_SEQUENCE(SEQ_STATE_IN_MAIN_LOOP); // Subsequent iterations
    }
    init_num += 1;
    if (init_num > SHOULE_INIT) {
        init();
        init_num = 0; // Reset init_num after re-initialization
    }
    /* USER CODE BEGIN 3 */    
    ProcessKeyPresses();

    if (!is_hot_start) {
        // 唤醒ADC进行采集
        Wakeup();
        
        // 等待ADC稳定和采集完成
        HAL_Delay(10); // 10ms采集时间
        
        // Normal operation mode: read ADC values
        temp1 = (float)adcx[1]*(3.3/4096);
        float raw_light_value_scaled = temp1*10;

        // Apply moving average filter
        light_value = apply_moving_average_filter(raw_light_value_scaled);
        servoAngle = light_value * (180.0/33.0);
        
        // 采集完成后让ADC休眠
        Sleep();
    } else {
        // This block is for the first iteration after a hot start.
        for (int i = 0; i < FILTER_WINDOW_SIZE; i++) {
            light_value_history[i] = light_value;
        }
        current_sum = light_value * FILTER_WINDOW_SIZE;
        num_samples_in_filter = FILTER_WINDOW_SIZE;
        filter_index = 0;

        is_hot_start = 0; // Clear hot start flag, run in normal mode afterwards
    }
    HAL_Delay(light_value / 10);
    printf("\r Light Sensor Value =%f\r",light_value);
    HAL_Delay(light_value / 10);
    printf("\r\n servoAngle = %f\r\n", servoAngle);
    // g_loop_check += 1;
    // if (g_loop_check >= 100) {
    //     validate_all_data_and_restore_state(1); // check data valid and repair?
    //     g_loop_check = 0; // Reset loop check counter every 100 iterations
    //     printf("Loop check: %d iterations completed.\r\n", g_loop_check);
    // }
    periodically_check_and_repair_backup_integrity();
    randomize_execution_order();
    // display_light_value_on_zlg7290(light_value);
    // I2C_ZLG7290_Write(&hi2c1, 0x70, ZLG_WRITE_ADDRESS1, Tx1_Buffer, 8);
    
    // // 修改舵机控制逻辑
    // if (first_loop_after_hot_start) {
    //     // 热启动第一次循环，跳过舵机控制，PWM值已经在启动时通过以上逻辑正确设置
    //     printf("Hot start: Skipping servo rotation in first loop\r\n");
    //     first_loop_after_hot_start = 0; // 清除标志，后续循环正常控制舵机
    // } else {
    //     // 正常情况下调用SteeringEngine_Rotate
    //     SteeringEngine_Rotate(servoAngle);
    // }

    // // Marquee LED control
    // if (g_marquee_logic_enabled && light_value > 20.0f) {
    //     Turn_Off_All_Marquee_LEDs();
    //     Turn_On_Marquee_LED(marquee_count % 4);
    //     marquee_count++;
    // } else {
    //     Turn_Off_All_Marquee_LEDs();
    // }

    // // Fan (DC Motor) control
    // if (g_fan_logic_enabled && light_value > 20.0f) {
    //     DC_Task(0x13);
    // } else {
    //     DC_Task(0x00);
    // }
    
    // // Buzzer control
    // if (light_value > 20.0f) {
    //     g_buzzer_enabled = 1;
    // } else {
    //     g_buzzer_enabled = 0;
    // }
    
    /***************************************/
    save_state();

    g_watchdog_feed_counter++;
    if (g_watchdog_feed_counter >= WATCHDOG_FEED_INTERVAL) { // Feed watchdog every 10 iterations
        Watchdog_Feed();
        g_watchdog_feed_counter = 0; // Reset feed counter
    }
    CHECK_SEQUENCE(SEQ_STATE_IN_MAIN_LOOP);
    
    // 剩余时间延时（总周期 - 采集时间）
    HAL_Delay(g_main_loop_delay_ms - 10); // 例如50ms - 10ms = 40ms休眠时间
  }
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