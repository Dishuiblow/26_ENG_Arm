/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "usart.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BMI088driver.h"
#include "kalman_filter.h"
#include "mahony_filter.h"
#include "bsp_dwt.h"
#include "BMI088Middleware.h"
#include "can_bsp.h"
#include "usb_device.h"
#include "read.h"
#include "CRC8_CRC16.h"
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
uint8_t data[8] = {0x00, 0x01, 0x02, 0x03, 0x14, 0x05, 0x06, 0x07};
uint8_t uart7_rx_buf[10];
uint8_t uart10_rx_buf[10];

__ALIGN_BEGIN uint8_t tx_buf_0309[64] __ALIGN_END;

osThreadId userUartTaskHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void StartUserUartTask(void const * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM3_Init();
  MX_FDCAN3_Init();
  MX_USART10_UART_Init();
  MX_UART7_Init();
  
  /* USER CODE BEGIN 2 */
  DWT_Init(480);
  
  /* BMI088初始化 */
  while (BMI088_init(&hspi2, 0) != BMI088_NO_ERROR)
  {
    ;
  }
  Power_OUT1_ON;
  Power_OUT2_ON;
  
  // 开启PC15，给UART扩展接口供电
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  
  FDCAN1_Config();
  FDCAN2_Config();

  MX_USB_DEVICE_Init();
  
  // 启动接收
  //HAL_UART_Receive_DMA(&huart10, uart10_rx_buf, sizeof(uart10_rx_buf));
  HAL_UART_Receive_IT(&huart7, uart7_rx_buf, sizeof(uart7_rx_buf));
  
  servo_init();
//  servo_PickUp();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* USER CODE BEGIN RTOS_THREADS */
  //osThreadDef(userUartTask, StartUserUartTask, osPriorityNormal, 0, 256);
 // userUartTaskHandle = osThreadCreate(osThread(userUartTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN|RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLL2.PLL2M = 24;
  PeriphClkInitStruct.PLL2.PLL2N = 200;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART7)
    {
        HAL_UART_Receive_IT(&huart7, uart7_rx_buf, sizeof(uart7_rx_buf));
    }
    else if(huart->Instance == USART10)
    {
        HAL_UART_Receive_DMA(&huart10, uart10_rx_buf, sizeof(uart10_rx_buf));
    }
}
*/
void StartUserUartTask(void const * argument)
{
  uint8_t seq = 0;             // 协议包序号
  uint16_t data_len = 30;      // 0x0309 数据段长度为 30 字节

  for(;;)
  {
    // =========================================================
    // 1. 保留 UART7 的发送逻辑 (阻塞发送)
    // =========================================================
    HAL_UART_Transmit(&huart7, data, 8, 100);

    // =========================================================
    // 2. 组装 0x0309 协议包 (包总长 = 包头5 + 命令2 + 数据30 + 校验2 = 39字节)
    // =========================================================
    
    // (1) 组装帧头 Frame Header (5 字节)
    tx_buf_0309[0] = 0xA5;                      // SOF 起始字节
    tx_buf_0309[1] = data_len & 0xFF;           // 数据长度低八位
    tx_buf_0309[2] = (data_len >> 8) & 0xFF;    // 数据长度高八位
    tx_buf_0309[3] = seq++;                     // 包序号累加
    Append_CRC8_Check_Sum(tx_buf_0309, 5);      // 计算帧头前4字节的CRC8并填入第5字节 tx_buf_0309[4]

    // (2) 组装命令码 Cmd ID (2 字节) -> 0x0309 (小端模式)
    tx_buf_0309[5] = 0x09; 
    tx_buf_0309[6] = 0x03;

    // (3) 填入自定义数据段 Data (30 字节)
    // 把你要发给上位机客户端的数据写进 tx_buf_0309[7] 到 tx_buf_0309[36]
    for(int i = 0; i < 30; i++) 
    {
        tx_buf_0309[7 + i] = 0x01 + i; // 这里作为示例，填充 0x01, 0x02...
    }

    // (4) 计算全包帧尾校验 Frame Tail (2 字节)
    Append_CRC16_Check_Sum(tx_buf_0309, 39);    // 计算全包前37字节的CRC16并附加到最后2字节

    // =========================================================
    // 3. 强制清洗 D-Cache (STM32H7 DMA 发送必加)
    // =========================================================
    SCB_CleanDCache_by_Addr((uint32_t *)tx_buf_0309, 64);

    // =========================================================
    // 4. 启动 DMA 发送给图传链路 (USART10)
    // =========================================================
    HAL_UART_Transmit_DMA(&huart10, tx_buf_0309, 39);

    // =========================================================
    // 5. 整体任务频率控制
    // =========================================================
    // 0x0309 频率上限为 10Hz，此处延时 100ms 既能满足图传要求，也能让 UART7 周期性发送
    osDelay(100); 
  }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
