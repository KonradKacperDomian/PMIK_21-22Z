/* USER CODE BEGIN Header */
/**
  ****************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @author			: Konrad Kacper Domian
  ****************************************************************************
  * @attention
  *
  *	The program was created as a final project forthe subject of PMIK,
  *	taught at the Faculty of Electronics and Information Technology,
  *	Warsaw University of Technology. The project was made in the winter semester
  *	of 2021-2022. The project is based on the development of the LIDAR scanner.
  *	ReadME and documentation can be found at the link:
  *
  *
  ****************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "vl53l1_api.h"
#include "X-NUCLEO-53L1A1.h"
#include "AR-3603HB_api.h"
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
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
VL53L1_Dev_t                   devCenter;
VL53L1_Dev_t                   devLeft;
VL53L1_Dev_t                   devRight;
VL53L1_DEV                     Dev = &devCenter;
int status;
volatile int IntCount;
#define HIGH_OF_SCAN 10
#define SCALING_OF_X_AXES 57
#define isAutonomousExample 1  /* Allow to select either autonomous ranging or fast ranging example */
#define isInterrupt 0 /* If isInterrupt = 1 then device working in interrupt mode, else device working in polling mode */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void MX_GPIO_Init(void);
void MX_USART2_UART_Init(void);
void MX_I2C1_Init(void);
void AutonomousLowPowerRangingTest(void); /* see Autonomous ranging example implementation in USER CODE BEGIN 4 section */
void InitializePeripherals(void);
void Reset3TofsensorOn53L1A1(uint8_t ToFSensor);
void SetNewI2CAdressOn53L1A1(uint8_t ToFSensor);
void TakeDataFrom53L1A1(uint8_t ToFSensor, VL53L1_RangingMeasurementData_t* RangingData);
void PrintDistanceOnTerminal(uint8_t ToFSensor, VL53L1_RangingMeasurementData_t* RangingData);
struct Voxel* Make3Dscan(uint8_t ToFSensor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
		if (GPIO_Pin==VL53L1X_INT_Pin)
		{
			IntCount++;
		}
}

struct Voxel
{
	float x;
	float y;
	float z;
};



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  uint8_t ToFSensor = 1; // 0=Left, 1=Center(default), 2=Right
  unsigned int StartScannig = 1;

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  InitializePeripherals();

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Reset3TofsensorOn53L1A1(ToFSensor);
  SetNewI2CAdressOn53L1A1(ToFSensor);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* USER CODE BEGIN 2 */
  InitAR3603HB();
  struct Voxel *Scan;
  if(StartScannig)
	  Scan = Make3Dscan(ToFSensor);
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* USER CODE BEGIN 4 */
/**
 * @brief InitializeAllPeripherals
 * @retval None
 */
void InitializePeripherals(void)
{
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	XNUCLEO53L1A1_Init();

}
/**
  * @brief  Take Data From 53L1A1 Laser Flying radar
  * @retval void
  */
void TakeDataFrom53L1A1(uint8_t ToFSensor, VL53L1_RangingMeasurementData_t* RangingData)
{
	for (ToFSensor=0;ToFSensor<3;ToFSensor++){
				switch(ToFSensor){
					case 0:
						Dev=&devLeft;
						break;
					case 1:
						Dev=&devCenter;
						break;
					case 2:
						Dev=&devRight;
					}
				status = VL53L1_StartMeasurement(Dev);
			  status = VL53L1_WaitMeasurementDataReady(Dev);
			}
}
/**
  * @brief  The scannig fun.
  * @retval pointer to struct Voxel
  */
struct Voxel* Make3Dscan(uint8_t ToFSensor)
{
	static VL53L1_RangingMeasurementData_t RangingData;
	static struct Voxel Scan[10+57];
	printf("Scanning\n");
	for(int j=0; j<HIGH_OF_SCAN; j++)
		{
	for(int i=0; i<SCALING_OF_X_AXES; i++)
	{
		TakeDataFrom53L1A1(ToFSensor, &RangingData);
		MoveAR3603HB(10, 1, isInterrupt);
		PrintDistanceOnTerminal(ToFSensor, &RangingData);
		Scan[j+i].x=i;
		Scan[j+i].y=RangingData.RangeMilliMeter;
		Scan[j+i].z=j;
	}
	MoveAR3603HB(100, 2, isInterrupt);

	}
	printf("Scanning finish\n");
	return Scan;
}
/**
  * @brief  Reset 53L1A1 Laser Flying radar
  * @retval void
  */
void Reset3TofsensorOn53L1A1(u_int8_t ToFSensor)
{
	for (ToFSensor=0;ToFSensor<3;ToFSensor++){
			status = XNUCLEO53L1A1_ResetId(ToFSensor, 0);
		}
}
/**
  * @brief  Print Data from 53L1A1 Laser Flying radar
  * @retval void
  */
void PrintDistanceOnTerminal(uint8_t ToFSensor, VL53L1_RangingMeasurementData_t* RangingData)
{
	if(!status)
					{
						status = VL53L1_GetRangingMeasurementData(Dev, RangingData);
						if(status==0){
							printf("%d,%d,%d,%.2f,%.2f\n", ToFSensor,RangingData->RangeStatus,RangingData->RangeMilliMeter, // @suppress("Float formatting support")
											(RangingData->SignalRateRtnMegaCps/65536.0),RangingData->AmbientRateRtnMegaCps/65336.0);
						}
						status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
					}
}
/**
  * @brief  Set adress I2C 53L1A1 Laser Flying radar
  * @retval void
  */
void SetNewI2CAdressOn53L1A1(uint8_t ToFSensor)
{
	uint16_t wordData;
	uint8_t newI2C = 0x52;
	for (ToFSensor=0;ToFSensor<3;ToFSensor++){
	  		switch(ToFSensor){
	  			case 0:
	  				Dev=&devLeft;
	  				break;
	  			case 1:
	  				Dev=&devCenter;
	  				break;
	  			case 2:
	  				Dev=&devRight;
	  				break;
	  		}
	  		status = XNUCLEO53L1A1_ResetId(ToFSensor, 1);
	  		Dev->comms_speed_khz = 400;
	  		Dev->I2cHandle = &hi2c1;
	  		Dev->comms_type = 1;
	  		Dev->I2cDevAddr=0x52; /* default ToF sensor I2C address*/
	  		VL53L1_RdWord(Dev, 0x010F, &wordData);
	  		printf("VL53L1X: %02X\n\r", wordData);
	  		newI2C = Dev->I2cDevAddr + (ToFSensor+1)*2;
	  		status = VL53L1_SetDeviceAddress(Dev, newI2C);
	  		Dev->I2cDevAddr=newI2C;
	  		VL53L1_RdWord(Dev, 0x010F, &wordData);
	  		printf("VL53L1X: %02X\n\r", wordData);
	  		/* Device Initialization and setting */
	  				status = VL53L1_WaitDeviceBooted(Dev);
	  				status = VL53L1_DataInit(Dev);
	  				status = VL53L1_StaticInit(Dev);
	  				status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
	  				status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
	  				status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 100);
	  			}
}

/* Autonomous ranging loop*/
void AutonomousLowPowerRangingTest(void)
{
  static VL53L1_RangingMeasurementData_t RangingData;
  status = VL53L1_WaitDeviceBooted(Dev);
  status = VL53L1_DataInit(Dev);
  status = VL53L1_StaticInit(Dev);
  status = VL53L1_SetDistanceMode(Dev, VL53L1_DISTANCEMODE_LONG);
  status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(Dev, 50000);
  status = VL53L1_SetInterMeasurementPeriodMilliSeconds(Dev, 500);
  status = VL53L1_StartMeasurement(Dev);

	if(status){
		printf("VL53L1_StartMeasurement failed \n");
		while(1);
	}	
	if (isInterrupt){
		do // interrupt mode
		{
		 __WFI();
		 if(IntCount !=0 ){
				IntCount=0;
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
					printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter, // @suppress("Float formatting support")
									RangingData.SignalRateRtnMegaCps/65536.0,RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
		}
		while(1);
	}
	else{
		do // polling mode
		{
		  status = VL53L1_WaitMeasurementDataReady(Dev);
			if(!status)
			{
				status = VL53L1_GetRangingMeasurementData(Dev, &RangingData);
				if(status==0){
					printf("%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,RangingData.RangeMilliMeter,
									(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
				}
				status = VL53L1_ClearInterruptAndStartMeasurement(Dev);
			}
		}
		while (1);
	}
//  return status;
}

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

