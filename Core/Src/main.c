/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You mAy not use this file except in compliance with
  * the License. You mAy obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

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
I2C_HandleTypeDef hi2c3;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19
#define GyRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GyRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int numberOfTests = 200;

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float   Axoff, Ayoff, Azoff; //accelerometer offset values
float   Gxoff, Gyoff, Gzoff; //Gyroscope offset values

float Ax, Ay, Az, Gx, Gy, Gz;
float M_PI = 3.14;

float   Roll,Pitch,Yaw;
float angle_Gx=0,angle_Gy=0,angle_Gz=0;
float angle_Ax=0,angle_Ay=0,angle_Az=0;
#define TIME_GyRO 2000

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GyRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, GyRO_CONFIG_REG, 1, &Data, 1, 1000);
	}

}

void test_values(void)
{
	char buf[100];
     for(int i =0; i<numberOfTests; i++)
    {
     sprintf(buf, "Test Number: %d \n\r", i);
     HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);

     MPU6050_Read_Accel();
     MPU6050_Read_Gyro();

      Axoff += Accel_X_RAW;
      Ayoff += Accel_Y_RAW;
      Azoff += Accel_Z_RAW;
      Gxoff += Gyro_X_RAW;
      Gyoff += Gyro_Y_RAW;
      Gzoff += Gyro_Z_RAW;

      sprintf(buf, "Ax:%d, Ay:%d, Az:%d || Gx:%d, Gy:%d, Gz:%d,\t", Accel_X_RAW,Accel_Y_RAW,
    		  Accel_Z_RAW,Gyro_X_RAW,Gyro_Y_RAW,Gyro_Z_RAW);
      HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);
      HAL_UART_Transmit(&huart2,(uint8_t*)"\n\r",4, 50);
      HAL_Delay(25);
    }
    Axoff = Axoff/numberOfTests;
    Ayoff = Ayoff/numberOfTests;
    Azoff = Azoff/numberOfTests;
    Gxoff = Gxoff/numberOfTests;
    Gyoff = Gyoff/numberOfTests;
    Gzoff = Gzoff/numberOfTests;

    sprintf(buf, "\n\nTest finished, offset values are shown below\n\n\r");
    HAL_UART_Transmit(&huart2,(uint8_t*)buf, sizeof(buf), 50);
    sprintf(buf, "Axoff:%d, Ayoff:%d, Azoff:%d || Gxoff:%d, Gyoff:%d, Gzoff:%d,\t", (int)Axoff,(int)Ayoff,(int)Azoff,(int)Gxoff,(int)Gyoff,(int)Gzoff);
    HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);
}


void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GyRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, GyRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (�/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GyRO_CONFIG Register              ****/

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */


  //lcd_init();

  MPU6050_Init();

  //lcd_send_string ("initialized");

  //HAL_Delay (1000);  // wait for 1 sec

  //lcd_clear ();

  //lcd_send_cmd (0x80|0x5A);
  //lcd_send_string ("MPU6050");

  test_values();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    /*if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)==GPIO_PIN_SET)
    {
    	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13, GPIO_PIN_SET);
    }
    else
    {
    	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
      }*/
	  // read the Accelerometer and Gyro values

	  MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();


	  Ax = (Accel_X_RAW - Axoff)/16384.0;
	  Ay = (Accel_Y_RAW - Ayoff)/16384.0;
      Az = (Accel_Z_RAW-(Azoff-16384))/16384.0;

  	  Gx = (Gyro_X_RAW - Gxoff)/131.0;
  	  Gy = (Gyro_Y_RAW - Gyoff)/131.0;
  	  Gz = (Gyro_Z_RAW - Gzoff)/131.0;

	  /*sprintf (buf, "Ax:%.2f | ", Ax);
	  //HAL_UART_Transmit(&huart2,"Hello",5, 10);
	  for( int i=0;i<sizeof(buf); i++)
		  HAL_UART_Transmit(&huart2,(uint8_t*)&buf[i],1, 50);

	  sprintf (buf, "Ay:%.2f | ", Ay);
	  for( int i=0;i<sizeof(buf); i++)
	  		  HAL_UART_Transmit(&huart2,(uint8_t*)&buf[i],1, 50);

	  sprintf (buf, "Az:%.2f ||| ", Az);
	  for( int i=0;i<sizeof(buf); i++)
	  		  HAL_UART_Transmit(&huart2,(uint8_t*)&buf[i],1, 50);*/

	  /*sprintf (buf, "Gx:%.2f | ", Gx);
	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);

	  sprintf (buf, "Gy:%.2f | ", Gy);
	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);

	  sprintf (buf, "Gz:%.2f \n\r", Gz);
	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);*/


	  //converting to angles
	  float FK = 0.9;
	  Roll  = atan2f(Ay, Az) * 180/M_PI;
	  Pitch = atan2f(Ax, sqrt(Ay*Ay + Az*Az)) * 180/M_PI;
	  //Yaw = atan2f(sqrt(Ay*Ay+Az*Az),Ax) * 180/M_PI;

	  angle_Ax = atan2(Ay,Az)*180/M_PI;
	  angle_Ay = atan2(Ax,Az)*180/M_PI;
	  angle_Az = atan2(Ay,Ax)*180/M_PI;


	  angle_Gx = angle_Gx + Gx*TIME_GyRO/1000000.0;
	  angle_Gy = angle_Gy + Gy*TIME_GyRO/1000000.0;
	  angle_Gz = angle_Gz + Gz*TIME_GyRO/1000000.0;

	  Yaw = angle_Gz*100;
	  //Комплементарний фільтр
	  angle_Ax = angle_Ax*FK + angle_Gx *(1-FK);
	  angle_Ay = angle_Ay*FK + angle_Gy *(1-FK);
	  angle_Az = angle_Az*FK + angle_Gz *(1-FK);

	  /*sprintf (buf, "angle_Ax:%.2f | ", angle_Ax);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);

	  	  sprintf (buf, "angle_Ay:%.2f | ", angle_Ay);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);

	  	  sprintf (buf, "angle_Az:%.2f \n\r", angle_Az);
	  	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 50);*/



//	  	  char buf[22];
//	  	  sprintf (buf, "%.2f %.2f %.2f\r\n", Roll,  Pitch,  Yaw);//angle_Az);
//	  	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf),  0xFFFF);

	  // simple test
	  char buf[6];
	  sprintf(buf, "Hello");
	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf),0xFFFF);


	  	//sprintf (buf, "Roll: %.2f; Pitch: %.2f; angle_Z: %.2f  \n\r", Roll,  Pitch,  angle_Az);
	  	//	  	  HAL_UART_Transmit(&huart2,(uint8_t*)buf,sizeof(buf), 10);

	  /*float control_num = 40;//control angle
	      if(angle_Ax>=control_num)//if(Roll>=control_num)
	      {
	          //Right
	    	  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n\nRight\n\n\r",18, 50);

	          HAL_Delay(500);
	      }
	      else if(angle_Ax<=-control_num)//else if(Roll<=-control_num)
	      {
	          //Left
	    	  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n\nLeft\n\n\r",18, 50);

	          HAL_Delay(500);
	      }
	      else if(angle_Ay>=control_num)//else if(Pitch>=control_num)
	      {
	          //Down
	    	  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n\nDown\n\n\r",18, 50);
	          HAL_Delay(500);
	      }
	      else if(angle_Ay<=-control_num)//else if(Pitch<=-control_num)
	      {
	          //Up
	    	  HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n\nUp\n\n\r",18, 50);
	          HAL_Delay(500);
	      }*/

	  HAL_Delay (250);  // wait for a while
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 200;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 5;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : DATA_Ready_Pin */
  GPIO_InitStruct.Pin = DATA_Ready_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DATA_Ready_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin PE13 */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INT1_Pin INT2_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = INT1_Pin|INT2_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
