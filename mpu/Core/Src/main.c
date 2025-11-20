/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


//TẠO BIẾN ĐỂ LƯU GIÁ TRỊ THÔ 16 BIT
int16_t Accel_X_RAW, Accel_Y_RAW,Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW,Gyro_Z_RAW;

//tạo biến lưu cứng giá trị sai số
float Accel_X_Offset = 0.0;
float Accel_Y_Offset = 0.0;
     //float Accel_Z_Offset = 0.0;
float Gyro_X_Offset = 0.0;
float Gyro_Y_Offset = 0.0;
float Gyro_Z_Offset = 0.0;

//tạo biến cho bộ lọc bù
float Accel_Pitch_Angle;
float Gyro_Pitch_Angle;
float Pitch_Angle = 0.0;

//biến thời gian
uint32_t last_time =0;
float dt = 0.0; // khoảng thời gian giữa 2 lần tính toán

// Hằng số bộ lọc
const float alpha = 0.98; // tin 98% vào GYRO
const float rad_to_deg = 180 / 3.14159265; // biến đổi rad sang độ

//biến cho PID
float kp = 110.0;
float ki = 0;
float kd = 12;

float Error, Last_Error = 0.0;
float P_Term, I_Term = 0.0 , D_Term;
float PID_speed;
float Setpoint = 0.00;
const float DEAD_ZONE_ANGLE = 45.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "stdio.h"
#include "math.h"
#include "stdlib.h"
#define MPU6050_ADDR (0x68 << 1) // 0xD0

// Địa chỉ các thanh ghi

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B

// hàm in dữ liệu qua UART
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)ptr,len, HAL_MAX_DELAY);
	return len;
}


void MPU6050_Init(void)
{
    uint8_t check;
    uint8_t data;

    // 1. Kiểm tra kết nối
    if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000) == HAL_OK)
    {
    		printf("chip ID = 0x%X\r\n",check);
    }else{
    	printf("I2C ERROR!\r\n");

    }



    // 2. Đánh thức và chọn Clock Gyro (RẤT QUAN TRỌNG)
    data = 0x01; // 0b00000001 -> CLKSEL = 1 (PLL với Gyro X)
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

    // 3. Cấu hình Sample Rate (Tốc độ lấy mẫu)
    // Sample Rate = Gyro Output Rate / (1 + SMPLRT_DIV)
    // 1kHz / (1 + 7) = 125Hz
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

    // 4. Cấu hình Bộ lọc thông thấp (DLPF)
    // Đặt DLPF_CFG = 3 -> Accel ~44Hz, Gyro ~42Hz. Giúp lọc nhiễu.
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, CONFIG_REG, 1, &data, 1, 1000);

    // 5. Cấu hình dải đo Gyro (+/- 250 dps)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);

    // 6. Cấu hình dải đo Accel (+/- 2g)
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

    printf("MPU6050 Configured!\r\n");
}

void MPU6050_Read_Data_Raw(void){

	uint8_t Rec_Data[14];
	// Đọc 14 byte
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR,0x3B, 1,Rec_Data,14,1000);

	// ráp 2 gói tin 8 bit thành 16 bit
	    Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	    Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	    Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	    // Bỏ qua 2 byte nhiệt độ (Rec_Data[6] và Rec_Data[7])
	    Gyro_X_RAW  = (int16_t)(Rec_Data[8]  << 8 | Rec_Data[9]);
	    Gyro_Y_RAW  = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	    Gyro_Z_RAW  = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	//hiệu chỉnh trừ sai số

}

void MPU6050_Calibrate(void){

	printf("bắt đầu calibrate, vui lòng giữ yên!\r\n");
	HAL_Delay(100);
	// Khỏ tạo biến chứa dữ liệu tính tổng
	int32_t Accel_X_Sum = 0;
	int32_t Accel_Y_Sum = 0;

	int32_t Gyro_X_Sum = 0;
	int32_t Gyro_Y_Sum = 0;
	int32_t Gyro_Z_Sum = 0;

	int num_samples = 1000;

	for(int i = 0; i < num_samples; i++){

		//đọc hàm Read_Data_Raw
		MPU6050_Read_Data_Raw();

		Accel_X_Sum += Accel_X_RAW;
		Accel_Y_Sum += Accel_Y_RAW;

		Gyro_X_Sum += Gyro_X_RAW;
		Gyro_Y_Sum += Gyro_Y_RAW;
		Gyro_Z_Sum += Gyro_Z_RAW;

		HAL_Delay(3); //chờ 3ms giữa các lần đọc tổng 3s

	}

	// tính trung bình ra tỉ lệ sai số offset và lưu vào biến toàn cục FLOAT
	Accel_X_Offset = (float) Accel_X_Sum / (float) num_samples;
	Accel_Y_Offset = (float) Accel_Y_Sum / (float) num_samples;

	Gyro_X_Offset = (float) Gyro_X_Sum /(float) num_samples;
	Gyro_Y_Offset = (float) Gyro_Y_Sum /(float) num_samples;
	Gyro_Z_Offset = (float) Gyro_Z_Sum /(float) num_samples;

	printf("đã hiệu chỉnh xong\r\n");
}



void MPU6050_Read_Data(void){

	MPU6050_Read_Data_Raw();
	        //hiệu chỉnh trừ sai số
	        Accel_X_RAW =Accel_X_RAW - Accel_X_Offset;
		    Accel_Y_RAW =Accel_Y_RAW - Accel_Y_Offset;
		    //Accel_Z_RAW =Accel_X_RAW - Accel_X_Offset;  KO CẦN DO DẪ SÁT THÔNG SỐ YÊU CÂU 1g
		    Gyro_X_RAW = Gyro_X_RAW - Gyro_X_Offset;
		    Gyro_Y_RAW = Gyro_Y_RAW - Gyro_Y_Offset;
		    Gyro_Z_RAW = Gyro_Z_RAW - Gyro_Z_Offset;
	}

void Motor_control(int speed_A, int speed_B){
	// Động cơ A
	if( speed_A > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}else if(speed_A < 0 ){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}

	// Dùng hàm macro vì cần set compare nhanh nhất có thể
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(speed_A));

	// Động cơ B
	if( speed_B > 0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		}else if(speed_B < 0 ){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		}else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		}

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, abs(speed_B));

}

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  // BẬT KÊNH PWM 1 VÀ 2 TRÊN L298N
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  MPU6050_Init();
  MPU6050_Calibrate();
  last_time = HAL_GetTick(); // khởi tạo mốc thời gian


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



    MPU6050_Read_Data();
    //printf("Gyro Z (RAW) = %d\r\n",Gyro_Z_RAW);
    //printf("Gyro X (RAW) = %d\r\n",Gyro_X_RAW);
    //printf("Gyro Y (RAW) = %d\r\n",Gyro_Y_RAW);
    //printf("Accel Z (RAW) = %d\r\n",Accel_Z_RAW);
    //printf("Accel X (RAW) = %d\r\n",Accel_X_RAW);
    //printf("Accel Y (RAW) = %d\r\n",Accel_Y_RAW);



    // ----tính khoảng thời gian dt---
    uint32_t curent_time = HAL_GetTick();
    dt = (float) ( curent_time - last_time) / 1000.0;
    last_time = curent_time;
    if (dt < 0.005) dt = 0.005;

    //tính góc Accel
    Accel_Pitch_Angle = atan2(Accel_X_RAW,Accel_Z_RAW) * rad_to_deg;

    //tính góc Gyro
    float Gyro_Y_Rate = Gyro_Y_RAW / 131.0;
    Gyro_Pitch_Angle = Pitch_Angle + (Gyro_Y_Rate * dt);

    //tính BỘ LỌC BÙ
    Pitch_Angle =  alpha * Gyro_Pitch_Angle + (1.0 - alpha)* Accel_Pitch_Angle;



    // VÙNG CHẾT - Robot không tự cứu khi nghiêng quá 45°
    if (fabs(Pitch_Angle) > DEAD_ZONE_ANGLE) {
        Motor_control(0, 0);
        I_Term = 0;  // Reset tích phân
        Last_Error = 0;
        printf("Angle: %.2f\r\n", Pitch_Angle);
        HAL_Delay(10);
        continue;
    }




    // tính toán P i D
   Error = Pitch_Angle - Setpoint;

   P_Term = kp * Error;

   I_Term = I_Term +(ki* Error * dt);

   D_Term = kd * (Error - Last_Error) /dt;

   PID_speed = P_Term + I_Term + D_Term;

   Last_Error = Error;


   // BÙ MA SÁT THÔNG MINH (Chỉ kích hoạt nếu lệnh đủ lớn)
       float min_speed = 660.0;
       float dead_band_filter = 10.0; // <-- Đặt ngưỡng an toàn (ví dụ: 20)

       if (fabs(PID_speed) < dead_band_filter) // Nếu PID quá nhỏ (nhỏ hơn 20)
       {
           PID_speed = 0; // COI NHƯ LỖI NHIỄU, TẮT motor
       }
       else
       {
           // Nếu PID đủ lớn để chống lại nhiễu, thì mới Bù Ma Sát
           if (PID_speed > 0) PID_speed += min_speed;
           else if (PID_speed < 0) PID_speed -= min_speed;
       }

        // Sau khi bù ma sát, thêm:
        if (PID_speed > 999) PID_speed = 999;
        if (PID_speed < -999) PID_speed = -999;

        // SET cứng I_TERM chống tràn
        if (I_Term > 1000) I_Term = 1000;
        if (I_Term < -1000) I_Term = -1000;

        // SET cứng I_TERM chống tràn
        if (D_Term > 999) D_Term = 999;
        if (D_Term <  -999) D_Term = -999;



   Motor_control(-(int)PID_speed, -(int)PID_speed);


       //in kết quả
       static uint32_t last_print_time = 0;
           if (curent_time - last_print_time >= 100) // In 10 lần/giây
           {
        	   // In ra Kp, Kd bạn đang "vặn"
        	   printf("Ang:%.1f Er:%.1f P:%d I:%d D:%d PWM:%d\r\n",Pitch_Angle, Error, (int)P_Term, (int)I_Term, (int)D_Term, (int)PID_speed);
               last_print_time = curent_time;
           }
           HAL_Delay(10);
/*
 // Test MOTOR
    Motor_control(600,600);
    HAL_Delay(2000);
    Motor_control(0,0);
    HAL_Delay(1000);
*/

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
