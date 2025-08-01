#ifndef USER_CONFIG_H
#define USER_CONFIG_H

/* Private enable--------------------------------------------------------------------------------*/
#define USE_STM32_DEMO 0 // USB test code enable
#define MAX_SAVE_TIMES 100
#define FILE_MAX_NAME_LEN 20

/* debug enable ---------------------------------------------------------------------------------*/
#define TEMP_ADJUST 0     // Temperature calibration enable
#define VOLTAGE_CHECK 1   // Overvoltage and undervoltage alarm enable
#define OVER_LOAD_CHECK 1 // Overload protection enable
#define POWER_ON_CHECK 1  // Boot detection enable

/* Algorithm enable ------------------------------------------------------------------------------*/
#define KALMAN_FILTER 0 // filter enable

/* Functional enable -----------------------------------------------------------------------------*/
#define SENSOR_CHECK_ENABLE 1
#define WRITE_CSV_ENABLE 0
#define EXPLORE_DISK_ENABLE 0
#define MODBUSSLAVE_ENABLE 1
#define START_IO_ENABLE 0
#define PID_DEBUG 0             // pid debug enable
#define COMMUNICATE 0           // Communication with the Upper computer enable
#define REALTIME_TEMP_DISPLAY 1 // Real-time temperature plotting enable
#define HOST_WELD_CTRL 1        // Upper computer control welding enable
#define RESERVE_CHECK_ENABLE 0
#define FAST_RISE_ENABLE 0

/* Temperature control ---------------------------------------------------------------------------*/

/*Temperature control parameters*/
#define STABLE_ERR_E 5              // Steady-state error compensation
#define STABLE_ERR_K 12              // Steady-state error compensation
#define STABLE_ERR_J 12              // Steady-state error compensation
#define FAST_RISE_TIME_DEFAULT 100   // Cold-start temperature compensation time
#define RISE_TIME_LIMIT 1500         // max of the rise time
#define DEFAULT_SLOPE 1.3            // dynamic algorithm coff
#define DEFAULT_INTERCEPT 1060       // dynamic algorithm coff
#define FINAL_DUTY_LIMIT 1800        // dynamic algorithm limit
#define DEFAULT_GAIN1 0.175          // Determine the compensation strength
#define DEFAULT_GAIN2 0.175          // Determine the compensation time
#define COMPENSATION_THRESHOLD 0.98f // Heat compensation temperature detection threshold
#define RESTRICT_TEMP_ERR 20         // restrict err
#define RESTRICT_BASE_COFF 0.1       // restrict output coff
#define MAX_START_TEMP 250           // max start temp

/* Temperature calibration macro------------------------------------------------------------------*/
#define ROOM_TEMP 20     // Default room temperature
#define SAMPLE_LEN 100   // Sampling depth
#define ADC_BIAS_MAX 650 // Maximum offset value

/* Dynamic correction macro-----------------------------------------------------------------------*/
#define PWM_SAMPLE 1                      // PWM sample enable
#define LEARNING_RATE 3                   // Determine the correction speed
#define MIN_STEP_SIZE 0.01f               // min correction speed
#define STABLE_THRESHOLD 30               // Pre-sampling filtering time
#define TEMP_STABLE_ERR 5                 // The permissible error allowed in steady-state sampling
#define STORAGE_DEPTH 500                 // Sampling storage depth
#define MAX_CORRECT_GAIN 1.25f            // max correct coefficient
#define MIN_CORRECT_GAIN 0.75f            // min correct coefficient
#define OVERSHOOT_THRESHOLD 1.05f         // The overshoot detection threshold(up)
#define REVERSE_OVERSHOOT_THRESHOLD 0.95f // The overshoot detection threshold(down)

/* plotting related macro-------------------------------------------------------------------------*/
#define WIN_WIDTH 525         // Display area width
#define DRAW_RESERVE 100      // Drawing margin
#define DRAW_AREA_HIGH 255    // Drawing component height
#define MAX_TEMP_DISPLAY 625  // Maximum displayable temperature
#define TEMP_BUF_MAX_LEN 4000 // Temperature acquisition buffer size

/* protect macro ---------------------------------------------------------------------------------*/
/*Thermocouple detection interval*/
#define KEJ_CHECK_DUTY 40
#define LOW_TEMP_LIMIT 5
/*Marco of check io define*/
#define CHECK_RCC_E RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_E GPIOE
#define CHECKOUT_PIN_E GPIO_Pin_3
#define CHECKIN_PIN_E GPIO_Pin_4

#define CHECK_RCC_J RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_J GPIOE
#define CHECKOUT_PIN_J GPIO_Pin_6
#define CHECKIN_PIN_J GPIO_Pin_7

#define CHECK_RCC_K RCC_AHB1Periph_GPIOE
#define CHECK_GPIO_K GPIOE
#define CHECKOUT_PIN_K GPIO_Pin_13
#define CHECKIN_PIN_K GPIO_Pin_14
/*UI PARAM ----------------------------------------------------------------------------------------*/
#define USER_FIRST_SET_MAX 650
#define USER_SET_MAX_TEMP 650.0 // User-set maximum temperature
#define USER_SET_MIN_TEMP 100   // User-set minimum temperature
#define USER_MAX_WELD_TIME 9999 // Maximum welding time
#define USER_MAX_COUNT 50000
#define MAX_GP 19

#endif
