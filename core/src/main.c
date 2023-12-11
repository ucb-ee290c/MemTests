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

/*
void LL_delay(uint64_t time) {
  uint64_t target_tick = CLINT->MTIME + (time * MTIME_FREQ);
  while (CLINT->MTIME < target_tick) {
    // asm("nop");
  }
}
*/

static uint64_t read_cycles() {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}


uint64_t test_cache(uint8_t prefetch_en) {
    volatile uint8_t *prefetch_en_reg = (volatile uint8_t *) 0x10050000;
    *prefetch_en_reg = prefetch_en;

    int NUM_TESTS = 10;

    int data_count;
    uint8_t *data;
    uint64_t start_t_stamp;
    uint64_t end_t_stamp;
    uint64_t access_time;
    int error;

    error = 0;
    start_t_stamp = read_cycles();
    for(int i = 0; i < NUM_TESTS; i++) {
      data_count = 0;
      data = ((uint8_t *)SCRATCH_BASE) + 3072;
      while (data_count < 10240) {
        *data = 194;
        volatile uint8_t data_loaded = *data;
        if (data_loaded != 194) {
          error += 1;
        }
        data += 1;
        data_count += 1;
      };
    }
    end_t_stamp = read_cycles();
    if (error > 0) {
      return 0;
    } else {
      return end_t_stamp - start_t_stamp;
    }
    
}

void LL_UART_transmit(UART_TypeDef *UARTx, const uint8_t *data, uint16_t size, uint32_t timeout) {
  while (size > 0) {
    while (READ_BITS(UARTx->TXDATA, UART_TXDATA_FULL_MSK)) {
      // return TIMEOUT;
    }
    UARTx->TXDATA = *data;
    data += sizeof(uint8_t);
    size -= 1;
  }
  return OK;
}

char str[64] = "Hello world from hart\n";
char str_wo_pre_pass[64] = "Mem PASS\n";
char str_wo_pre_error[64] = "Mem ERROR\n";
char str_w_pre_pass[64] = "Mem Prefetch PASS\n";
char str_w_pre_error[64] = "Mem Prefetch ERROR\n";
char str_pre_le[64] = "Mem Prefetch Less\n";
char str_pre_eq[64] = "Mem Prefetch Equal\n";
char str_pre_ge[64] = "Mem Prefetch Greater\n";
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

  // set up GPIO registers
  // GPIO_InitTypeDef GPIO_init_config;
  // GPIO_init_config.mode = GPIO_MODE_OUTPUT;
  // GPIO_init_config.pull = GPIO_PULL_NONE;
  // GPIO_init_config.drive_strength = GPIO_DS_STRONG;
  // HAL_GPIO_init(GPIOA, &GPIO_init_config, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

  // set up UART registers
  // UART_InitTypeDef UART_init_config;
  // UART_init_config.baudrate = 115200;
  // UART_init_config.mode = UART_MODE_TX_RX;
  // UART_init_config.stopbits = UART_STOPBITS_2;
  // HAL_UART_init(UART0, &UART_init_config);
  
  CLEAR_BITS(UART0->RXCTRL, UART_RXCTRL_RXEN_MSK);
  CLEAR_BITS(UART0->TXCTRL, UART_TXCTRL_TXEN_MSK);

  SET_BITS(UART0->RXCTRL, UART_RXCTRL_RXEN_MSK);
  SET_BITS(UART0->TXCTRL, UART_TXCTRL_TXEN_MSK);

  CLEAR_BITS(UART0->TXCTRL, UART_TXCTRL_NSTOP_MSK);
  CLEAR_BITS(UART0->TXCTRL, UART_STOPBITS_2);
  
  // baudrate setting
  // f_baud = f_sys / (div + 1)
  // UART0->DIV = (SYS_CLK_FREQ / 115200) - 1;
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    uint64_t mhartid = READ_CSR("mhartid");
    LL_UART_transmit(UART0, str, 64, 0);

    uint8_t prefetch_en = 0;
    uint64_t t_wo_prefetch;
    uint64_t t_w_prefetch;

    t_wo_prefetch = test_cache(prefetch_en);
    if (t_wo_prefetch == 0) {
      LL_UART_transmit(UART0, str_wo_pre_error, 64, 0);
    } else {
      LL_UART_transmit(UART0, str_wo_pre_pass, 64, 0);
    }
    prefetch_en = 1;
    t_w_prefetch = test_cache(prefetch_en);
    if (t_w_prefetch == 0) {
      LL_UART_transmit(UART0, str_w_pre_error, 64, 0);
    } else {
      LL_UART_transmit(UART0, str_w_pre_pass, 64, 0);
    }

    /*
    if (t_w_prefetch > (t_wo_prefetch / 2)) {
      LL_UART_transmit(UART0, str_pre_no_eff, 64, 0);
    } else {
      LL_UART_transmit(UART0, str_pre_eff, 64, 0);
    }
    
    if (t_w_prefetch > (t_wo_prefetch / 100)) {
      LL_UART_transmit(UART0, str_pre_no_eff, 64, 0);
    } else {
      LL_UART_transmit(UART0, str_pre_eff, 64, 0);
    }
    */
    if (t_w_prefetch < t_wo_prefetch) {
      LL_UART_transmit(UART0, str_pre_le, 64, 0);
    } else if (t_w_prefetch == t_wo_prefetch) {
      LL_UART_transmit(UART0, str_pre_eq, 64, 0);
    } else {
      LL_UART_transmit(UART0, str_pre_ge, 64, 0);
    }

    counter += 1;

    //LL_delay(1000);
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
