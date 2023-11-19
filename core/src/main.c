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
void test_cache() {
    int NUM_TESTS = 10;

    int data_count;
    uint8_t *data;
    uint64_t start_t_stamp;
    uint64_t end_t_stamp;
    uint64_t access_time;
    int error;

    error = 0;
    for(int i = 0; i < NUM_TESTS; i++) {
      data_count = 0;
      data = (uint8_t *) (SRAM_BASE + 61440);
      while (data_count < 1024) {
        volatile *data = 194;
        volatile uint8_t data_loaded = *data;
        if (data_loaded != 194) {
          error += 1;
        }
        data += 1;
        data_count += 1;
      };
    }
    printf("Average Cache Sequential Read after Write Error: %d\n", error);
    
    access_time = 0;
    for(int i = 0; i < NUM_TESTS; i++) {
      data_count = 0;
      data = (uint8_t *) (SRAM_BASE + 61440);
      start_t_stamp = HAL_CLINT_getTime(); 
      while (data_count < 1024) {
        volatile uint8_t data_loaded = *data;
        data += 1;
        data_count += 1;
      };
      end_t_stamp = HAL_CLINT_getTime();
      access_time += end_t_stamp - start_t_stamp;
    }
    printf("Average Cache Sequential Read Time: %d\n", access_time / NUM_TESTS);
    
    access_time = 0;
    for(int i = 0; i < NUM_TESTS; i++) {
      data_count = 0;
      data = (uint8_t *) (SRAM_BASE + 61440);
      start_t_stamp = HAL_CLINT_getTime(); 
      while (data_count < 1024) {
        volatile *data = 194;
        data += 1;
        data_count += 1;
      };
      end_t_stamp = HAL_CLINT_getTime();
      access_time += end_t_stamp - start_t_stamp;
    }
    printf("Average Cache Sequential Write Time: %d\n", access_time / NUM_TESTS);
}


void test_scratchpad() {
  uint64_t start_t_stamp;
  uint64_t end_t_stamp;
  uint64_t access_time;

  start_t_stamp = HAL_CLINT_getTime(); 

  llist_t root;
  llist_t *cursor = &root;
  int count = 0;

  while (count < 1024) {
    cursor->val = 194;
    llist_t next;
    cursor->next = &next;
    cursor = &next;
  }

  end_t_stamp = HAL_CLINT_getTime();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(int argc, char **argv) {
  /* USER CODE BEGIN 1 */
  uint8_t counter = 0;
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  GPIO_InitTypeDef GPIO_init_config;
  GPIO_init_config.mode = GPIO_MODE_OUTPUT;
  GPIO_init_config.pull = GPIO_PULL_NONE;
  GPIO_init_config.drive_strength = GPIO_DS_STRONG;
  HAL_GPIO_init(GPIOA, &GPIO_init_config, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

  UART_InitTypeDef UART_init_config;
  UART_init_config.baudrate = 115200;
  UART_init_config.mode = UART_MODE_TX_RX;
  UART_init_config.stopbits = UART_STOPBITS_2;
  HAL_UART_init(UART0, &UART_init_config);

  // delay 5 seconds so if user code breaks chip JTAG can still gain control within the 5 second
  HAL_delay(5000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    HAL_GPIO_writePin(GPIOA, GPIO_PIN_2, 1);
    HAL_delay(100);

    HAL_GPIO_writePin(GPIOA, GPIO_PIN_2, 0);
    HAL_delay(100);


    /* 
    char str[128];
    uint64_t mhartid = READ_CSR("mhartid");
    sprintf(str, "Hello world from hart %d: %d\n", mhartid, counter);
    HAL_UART_transmit(UART0, (uint8_t *)str, strlen(str), 100);
    counter += 1;
    */

   /* Tests for L2 Cache */
   //TODO: Disable MTE
   //TODO: Disable Prefetcher
    test_cache();

    /* Tests for Prefetcher */
    //TODO: Disable MTE
    //TODO: Enable Prefetcher
    //test_cache();

    /* Tests for MTE */
    //TODO: Enable MTE
    //TODO: Disable Prefetcher
    //test_cache();

    /* Tests for ScratchPad */
    //TODO: Disable MTE
    //TODO: Disable Prefetcher
    //test_scratchpad();

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
