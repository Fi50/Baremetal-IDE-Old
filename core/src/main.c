/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float pd_controller(float curr_theta, float curr_x, float curr_dtheta, float curr_dx) {
    const float kp_theta = -0.012; 
    // const float kd_theta = -0.015;
    ///const float kd_theta = -0.0012;
    const float kd_theta = -0.0012;
    const float kp_x = 0.000;
    // const float kd_x = -0.01;
    const float kd_x = -0.0080;

    float p_term_theta = kp_theta * (-curr_theta); 
    float d_term_theta = kd_theta * curr_dtheta;

    float p_term_x = kp_x * (-curr_x); 
    float d_term_x = kd_x * curr_dx;

    float control_output_theta = p_term_theta - d_term_theta;
    float control_output_x = p_term_x - d_term_x;

    return control_output_theta + control_output_x;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  /* USER CODE BEGIN 1 */
  uint8_t counter = 0;
  const uint8_t obs_dim = 4;
  const uint8_t control_dim = 1;
  float obs_buffer[4] = {0.0, 0.0, 0.0, 0.0};
  float control_buffer[1] = {0.0};
  const uint8_t size = 10;
    
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  // set up UART registers
  UART_InitTypeDef UART_init_config;
  UART_init_config.baudrate = 115200;
  UART_init_config.mode = UART_MODE_TX_RX;
  UART_init_config.stopbits = UART_STOPBITS_2;
  HAL_UART_init(UART0, &UART_init_config);

  // GPIO_InitTypeDef gpiosettings;
  // gpiosettings.mode = GPIO_MODE_OUTPUT;
  // gpiosettings.pull = GPIO_PULL_NONE;
  // gpiosettings.drive_strength = GPIO_DS_STRONG;
  // HAL_GPIO_init(GPIOA, &gpiosettings, GPIO_PIN_0);

  // HAL_GPIO_writePin(GPIOA, GPIO_PIN_0, 0);
  // HAL_delay(500);

  // HAL_GPIO_writePin(GPIOA, GPIO_PIN_0, 1);
  // HAL_delay(500);
  // HAL_GPIO_writePin(GPIOA, GPIO_PIN_0, 0);
  // HAL_delay(5000);


  uint8_t synch;
  uint8_t magic = 123;
  while(!(synch == 123 )) {
    HAL_UART_receive(UART0, &synch, 1, 0);
  }

  HAL_UART_transmit(UART0, &magic, 1, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint64_t mhartid = READ_CSR("mhartid");

    // printf("Hello world from hart %d: %d\n", mhartid, counter);

    HAL_UART_receive(UART0, &obs_buffer, obs_dim * sizeof(float), 0);
    control_buffer[0] = pd_controller(obs_buffer[0], obs_buffer[1], obs_buffer[2], obs_buffer[3]);
    HAL_UART_transmit(UART0, &control_buffer, control_dim * sizeof(float), 0);
    
    counter += 1;

    /* USER CODE END WHILE */
  }
  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/*
 * Main function for secondary harts
 * 
 * Multi-threaded programs should provide their own implementation.
 */
void __attribute__((weak, noreturn)) __main(void) {
  while (1) {
   asm volatile ("wfi");
  }
}
