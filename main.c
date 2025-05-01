#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "ssd1306.h"
#include <stdio.h>
#include <string.h>
#define DHT11_GPIO_PORT GPIOA
#define DHT11_PIN GPIO_PIN_1
GPIO_InitTypeDef gpio={0};
#include "core_cm3.h"  // 根据你的内核选择头文件（CM3/CM4/CM7）


void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 启用跟踪
    DWT->CYCCNT = 0;                               // 计数器清零
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;           // 启用CYCCNT
}
void Send_Message(const char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void DHT11_Reset_Bus() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    HAL_GPIO_Init(GPIOA, &gpio);
}

char uart_buffer[128]; 
void send_float_via_uart(UART_HandleTypeDef *huart, float value, const char *name) {
    // 将浮点数和名称格式化为字符串
    int len = sprintf(uart_buffer, "%s: %.2f\r\n", name, value);
    
    // 通过UART发送
    if (len > 0) {
        HAL_UART_Transmit(huart, (uint8_t *)uart_buffer, len, 100);  // 100ms 超时
    }
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t cycles = us * (SystemCoreClock / 1000000); // 计算需要的时钟周期数
    while ((DWT->CYCCNT - start) < cycles);
}
/*void delay_us(uint32_t us) {
    TIM2->CNT = 0; // 计数器清零
    while (TIM2->CNT < us);
}*/

// 发送开始信号（主机拉低18ms后拉高20-40us）
void DHT11_Start() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_Delay(18);  // 保持18ms低电平
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    delay_us(30);   // 拉高20-40us（需自定义微秒延时函数）
    gpio.Mode = GPIO_MODE_INPUT; // 切换为输入模式等待响应
	gpio.Pull = GPIO_PULLUP; // 明确启用上拉
    HAL_GPIO_Init(GPIOA, &gpio);
}

uint8_t DHT11_Read_Byte() {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        while (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)); // 等待50us低电平开始
        delay_us(33); // 判断高电平持续时间
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) 
            data |= (1 << (7 - i)); // 高电平>30us为1
        while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)); // 等待位结束
    }
    return data;
}


// 检测DHT11响应信号
uint8_t DHT11_Check_Response() {
	
    uint8_t response = 0;
    delay_us(40);
    if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) { // 检测80us低电平
        delay_us(80);
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) { // 检测80us高电平
            response = 1;
        }
    }
    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)); // 等待高电平结束
	
    return response;
}


HAL_StatusTypeDef DHT11_Read_Data(float *temp, float *humi) {
    uint8_t data[5] = {0};
	
    DHT11_Start();
	//Send_Message("next3\r\n");
    if (!DHT11_Check_Response()) {return HAL_ERROR; } // DHT11 无响应
	//Send_Message("next3\r\n");
	for (int i = 0; i < 5; i++) 
            data[i] = DHT11_Read_Byte();
	
	if (data[4] != (data[0] + data[1] + data[2] + data[3])) {
            return HAL_ERROR;
        }
	
	*humi = data[0] + data[1] * 0.1;
    *temp = data[2] + data[3] * 0.1;
	
	return HAL_OK;
}



void SystemClock_Config(void);
int main(void)
{
	// 初始化代码
	__HAL_RCC_GPIOA_CLK_ENABLE();  

	
	DWT_Init();
   HAL_Init();

    SystemClock_Config();

    MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
	
	SSD1306_Init(&hi2c1);
	SSD1306_Clear();
	
	gpio.Pin = GPIO_PIN_1;
	gpio.Mode = GPIO_MODE_OUTPUT_OD;  // 开漏输出
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // 初始高电平
	float temp=0, humi=0;
    while (1)
  {
	  //DHT11_Reset_Bus();  // 确保总线状态恢复
	  //Send_Message("next1\r\n");
    if (DHT11_Read_Data(&temp, &humi) == HAL_OK) {
		//Send_Message("next2\r\n");
    send_float_via_uart(&huart1, temp, "temp");
		SSD1306_Print(0,0,"temp:");
		SSD1306_PrintFloat(60,0,temp,2);
		
	send_float_via_uart(&huart1, humi, "humi");
		SSD1306_Print(0,2,"humi:");
		SSD1306_PrintFloat(60,2,humi,2);
	} else {
    Send_Message("DHT11 Read Failed!\r\n");
	}
	
	DHT11_Reset_Bus(); // 每次读取后重置总线
	  HAL_Delay(2000); // 2秒读取一次
  }
  
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
