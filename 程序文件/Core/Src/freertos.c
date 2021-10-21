/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_5   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_6  +  GetSectorSize(ADDR_FLASH_SECTOR_6) -1 /* End @ of user Flash area : sector start address + sector size -1 */

#define DATA_32                 ((uint32_t)0)

uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);


/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;

  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
}


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

uint32_t count=0;
uint16_t adc_filter[11];
uint16_t adc_first[11];
int adc_one[11];
int foc[6];

int foc_cut[6];
int16_t instruct[500][11]=
{{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,0,0,0,0},
{0,0,0x3f,0x3f,0,0,0,0,0,0,0},
{0,0,0x3f,0x3f,500,0,0,0,0,0,0},
{0,0,0x1f,0x3f,600,0,0,0,0,0,0},
{0,0,0x3f,0x3f,600,500,0,0,0,0,0},
{0,0,0x1f,0x3f,600,600,0,0,0,0,0},
{0,0,0x3f,0x3f,600,600,500,0,0,0,0},
{0,0,0x1f,0x3f,600,600,600,0,0,0,0},
{0,0,0x3f,0x3f,0,0,0,0,0,0,0},
{0,0,0x1f,0x3f,0,0,0,0,0,0,0},
{0,0,0x3f,0x3f,0,0,0,0,0,0,0},
{0,0,0x1f,0x3f,0,0,0,0,0,0,0},
{0,0,0x3f,0x3f,0,0,0,0,0,0,0},
{0,0,0x1f,0x3f,0,0,0,0,0,0,0},
{0,0,0x2f,0x3f,0,0,0,0,0,0,0},
{0,0,0x1f,0x3f,0,0,0,0,0,0,0},
{0,0,0x0f,0x3f,0,0,0,0,0,0,0}};//模式，条件|输出，速度，加速度，位置，时间延时


int instruct_step=10;
int total_step =500;
float step=0;
float step_speed=0;
float step_acc=0;
float step_time=0;
char is_start=0;
char is_stop=0;
char is_single_run=0;
char is_record=0;
//电机1参数
int tar_pulse1=0;
int cur_pulse1=0;
int pre_pulse1=0;
int tar_speed1=0;
int cur_speed1=0;
int tar_acc1=0;
int cur_acc1=0;

//电机2参数
int tar_pulse2=0;
int cur_pulse2=0;
int pre_pulse2=0;
int tar_speed2=0;
int cur_speed2=0;
int tar_acc2=0;
int cur_acc2=0;

//电机3参数
int tar_pulse3=0;
int cur_pulse3=0;
int tar_speed3=0;
int cur_speed3=0;
int tar_acc3=0;
int cur_acc3=0;

//电机4参数
int tar_pulse4=0;
int cur_pulse4=0;
int tar_speed4=0;
int cur_speed4=0;
int tar_acc4=0;
int cur_acc4=0;

//电机5参数
int tar_pulse5=0;
int cur_pulse5=0;
int tar_speed5=0;
int cur_speed5=0;
int tar_acc5=0;
int cur_acc5=0;

//电机6参数
int tar_pulse6=0;
int cur_pulse6=0;
int tar_speed6=0;
int cur_speed6=0;
int tar_acc6=0;
int cur_acc6=0;


unsigned short CRC16_Modbus ( unsigned char *pdata, int len)
{
	unsigned short crc=0xFFFF;
	int i, j;
	for ( j=0; j<len;j++)
	{
		crc=crc^pdata[j];
		for ( i=0; i<8; i++)
		{
			if( ( crc&0x0001) >0)
			{
				crc=crc>>1;
				crc=crc^ 0xa001;
			}
			else
				crc=crc>>1;
		}
	}
	return crc;
}


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LedTaskHandle;
osThreadId SendTaskHandle;
osThreadId RobotTaskHandle;
osThreadId DemoTaskHandle;
osThreadId TestTaskHandle;
osThreadId ModbusTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartLedTask(void const * argument);
void StartSendTask(void const * argument);
void StartRobotTask(void const * argument);
void StartDemoTask(void const * argument);
void StartTestTask(void const * argument);
void StartModbusTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LedTask */
  osThreadDef(LedTask, StartLedTask, osPriorityNormal, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* definition and creation of SendTask */
  osThreadDef(SendTask, StartSendTask, osPriorityNormal, 0, 512);
  SendTaskHandle = osThreadCreate(osThread(SendTask), NULL);

  /* definition and creation of RobotTask */
  osThreadDef(RobotTask, StartRobotTask, osPriorityNormal, 0, 256);
  RobotTaskHandle = osThreadCreate(osThread(RobotTask), NULL);

  /* definition and creation of DemoTask */
  osThreadDef(DemoTask, StartDemoTask, osPriorityNormal, 0, 128);
  DemoTaskHandle = osThreadCreate(osThread(DemoTask), NULL);

  /* definition and creation of TestTask */
  osThreadDef(TestTask, StartTestTask, osPriorityIdle, 0, 128);
  TestTaskHandle = osThreadCreate(osThread(TestTask), NULL);

  /* definition and creation of ModbusTask */
  osThreadDef(ModbusTask, StartModbusTask, osPriorityNormal, 0, 512);
  ModbusTaskHandle = osThreadCreate(osThread(ModbusTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */


  /* Infinite loop */
  for(;;)
  {
  	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
//  	HAL_GPIO_TogglePin(STEP4_PUL_GPIO_Port, STEP4_PUL_Pin);
    osDelay(100);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartSendTask */
/**
* @brief Function implementing the SendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendTask */
void StartSendTask(void const * argument)
{
  /* USER CODE BEGIN StartSendTask */
  /* Infinite loop */
  for(;;)
  {

  	instruct[0][4] = cur_pulse1*0.6338;
  	instruct[0][5] = cur_pulse2*0.67164;
  	instruct[0][6] = cur_pulse3*1.13924;
  	instruct[0][7] = cur_pulse4*1.56673;
  	instruct[0][8] = cur_pulse5*2.45108;
  	instruct[0][9] = cur_pulse6;


  	instruct[2][4] = (instruct[2][2]+10)*11;
  	instruct_step = instruct[2][2]+10;
  	total_step = instruct[2][1];
    osDelay(10);
  }
  /* USER CODE END StartSendTask */
}

/* USER CODE BEGIN Header_StartRobotTask */
/**
* @brief Function implementing the RobotTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRobotTask */
void StartRobotTask(void const * argument)
{
  /* USER CODE BEGIN StartRobotTask */
	osDelay(1000);

	Address = FLASH_USER_START_ADDR;
	for(int j=11;j<(500*11);j++)
	{
		instruct[j/11][j%11] = (*(__IO int16_t*)Address);
		Address = Address + 2;
	}
	instruct[1][0]=0;
	instruct[1][1]=0;
	instruct[1][2]=0;
	instruct[1][3]=0;
	instruct[1][4]=0;
	instruct[1][5]=0;
	instruct[1][6]=0;
	instruct[1][7]=0;
	instruct[1][8]=0;
	instruct[1][9]=0;
	instruct[1][10]=0;
	instruct[2][0]=0;
	instruct[2][2]=0;




	is_start = 0;
  /* Infinite loop */
  for(;;)
  {
  	if(is_start==0)
  	{
  		if(instruct[0][0]==0)
  		{
  			for(int i=0;i<6;i++)
  			{
  				if(foc_cut[i]>-10)foc_cut[i]-=10;
  				if(foc_cut[i]<10)foc_cut[i]+=10;
  			}
  		}
  		else if(instruct[0][0]==1&&foc_cut[0]<3000)foc_cut[0]++;
  		else if(instruct[0][0]==2&&foc_cut[0]>-3000)foc_cut[0]--;
  		else if(instruct[0][0]==4&&foc_cut[1]<3000)foc_cut[1]++;
  		else if(instruct[0][0]==8&&foc_cut[1]>-3000)foc_cut[1]--;
  		else if(instruct[0][0]==16&&foc_cut[2]<3000)foc_cut[2]++;
  		else if(instruct[0][0]==32&&foc_cut[2]>-3000)foc_cut[2]--;
  		else if(instruct[0][0]==64&&foc_cut[3]<3000)foc_cut[3]++;
  		else if(instruct[0][0]==128&&foc_cut[3]>-3000)foc_cut[3]--;
  		else if(instruct[0][0]==256&&foc_cut[4]<3000)foc_cut[4]++;
  		else if(instruct[0][0]==512&&foc_cut[4]>-3000)foc_cut[4]--;
  		else if(instruct[0][0]==1024&&foc_cut[5]<3000)foc_cut[5]++;
  		else if(instruct[0][0]==2048&&foc_cut[5]>-3000)foc_cut[5]--;

  		if(instruct[2][0]==0)
  		{
  			is_single_run = 0;
  		}
  		else if(instruct[2][0]==1)
  		{

  			instruct[instruct_step][4] = cur_pulse1;
  			instruct[instruct_step][5] = cur_pulse2;
  			instruct[instruct_step][6] = cur_pulse3;
  			instruct[instruct_step][7] = cur_pulse4;
  			instruct[instruct_step][8] = cur_pulse5;
  			instruct[instruct_step][9] = cur_pulse6;
  		}
  		else if(instruct[2][0]==2)
  		{
  			is_single_run = 1;
  		}

  		else if(instruct[2][0]==4)
  		{
  		  /* Unlock the Flash to enable the flash control register access *************/
  		  HAL_FLASH_Unlock();

  		  /* Erase the user Flash area
  		    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  		  /* Get the 1st sector to erase */
  		  FirstSector = GetSector(FLASH_USER_START_ADDR);
  		  /* Get the number of sector to erase from 1st sector*/
  		  NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

  		  /* Fill EraseInit structure*/
  		  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  		  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  		  EraseInitStruct.Sector = FirstSector;
  		  EraseInitStruct.NbSectors = NbOfSectors;
  		  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  		  {
  		    while (1)
  		    {
  		    	osDelay(1000);
  		    	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  		    }
  		  }


  		  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
  		      you have to make sure that these data are rewritten before they are accessed during code
  		      execution. If this cannot be done safely, it is recommended to flush the caches by setting the
  		      DCRST and ICRST bits in the FLASH_CR register. */
  		   __HAL_FLASH_DATA_CACHE_DISABLE();
  		   __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  		   __HAL_FLASH_DATA_CACHE_RESET();
  		   __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  		   __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  		   __HAL_FLASH_DATA_CACHE_ENABLE();

  		   /* Program the user Flash area word by word
  		     (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  		   Address = FLASH_USER_START_ADDR;
  		   int instruct_addr = 11;
  		   int instruct_data = 0;
  		   while (Address < FLASH_USER_END_ADDR)
  		   {
  		  	 if(instruct_addr<(11*500))
  		  	 {
  		  		 instruct_data = instruct[instruct_addr/11][instruct_addr%11];
  		  	 }
  		  	 else instruct_data = 0;

  		     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, instruct_data) == HAL_OK)
  		     {
  		    	 instruct_addr++;
  		       Address = Address + 2;
  		     }
  		     else
  		     {
  		       /* Error occurred while writing data in Flash memory.
  		          User can add here some code to deal with this error */
  		       while (1)
  		       {
    		       	osDelay(500);
    		       	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  		       }
  		     }
  		   }


  		   /* Lock the Flash to disable the flash control register access (recommended
  		      to protect the FLASH memory against possible unwanted operation) *********/
  		   HAL_FLASH_Lock();
  		}

  		else if(instruct[2][0]==8)
  		{
  		  /* Unlock the Flash to enable the flash control register access *************/
  		  HAL_FLASH_Unlock();

  		  /* Erase the user Flash area
  		    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  		  /* Get the 1st sector to erase */
  		  FirstSector = GetSector(FLASH_USER_START_ADDR);
  		  /* Get the number of sector to erase from 1st sector*/
  		  NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

  		  /* Fill EraseInit structure*/
  		  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  		  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  		  EraseInitStruct.Sector = FirstSector;
  		  EraseInitStruct.NbSectors = NbOfSectors;
  		  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  		  {
  		    while (1)
  		    {
  		    	osDelay(1000);
  		    	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  		    }
  		  }


  		  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
  		      you have to make sure that these data are rewritten before they are accessed during code
  		      execution. If this cannot be done safely, it is recommended to flush the caches by setting the
  		      DCRST and ICRST bits in the FLASH_CR register. */
  		   __HAL_FLASH_DATA_CACHE_DISABLE();
  		   __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  		   __HAL_FLASH_DATA_CACHE_RESET();
  		   __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  		   __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  		   __HAL_FLASH_DATA_CACHE_ENABLE();

  		   /* Program the user Flash area word by word
  		     (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

  		   Address = FLASH_USER_START_ADDR;
  		   while (Address < FLASH_USER_END_ADDR)
  		   {
  		     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, 0) == HAL_OK)
  		     {
  		       Address = Address + 2;
  		     }
  		     else
  		     {
  		       /* Error occurred while writing data in Flash memory.
  		          User can add here some code to deal with this error */
  		       while (1)
  		       {
    		       	osDelay(500);
    		       	HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  		       }
  		     }
  		   }


  		   /* Lock the Flash to disable the flash control register access (recommended
  		      to protect the FLASH memory against possible unwanted operation) *********/
  		   HAL_FLASH_Lock();
  		}



  	}

		if(instruct[3][0]&0x01)
		{
			is_start = 1;
		}
		else is_start = 0;

		if(instruct[3][0]&0x02)
		{
			is_stop = 1;
			is_start = 0;
		}
		else is_stop = 0;



    osDelay(1);
  }
  /* USER CODE END StartRobotTask */
}

/* USER CODE BEGIN Header_StartDemoTask */
/**
* @brief Function implementing the DemoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDemoTask */
void StartDemoTask(void const * argument)
{
  /* USER CODE BEGIN StartDemoTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDemoTask */
}

/* USER CODE BEGIN Header_StartTestTask */
/**
* @brief Function implementing the TestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTestTask */
void StartTestTask(void const * argument)
{
  /* USER CODE BEGIN StartTestTask */
  /* Infinite loop */
  for(;;)
  {
  	HAL_UART_Receive_DMA(&huart2, rx_data, rx_bufSize);
    osDelay(1000);
  }
  /* USER CODE END StartTestTask */
}

/* USER CODE BEGIN Header_StartModbusTask */
/**
* @brief Function implementing the ModbusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartModbusTask */
void StartModbusTask(void const * argument)
{
  /* USER CODE BEGIN StartModbusTask */
	uint8_t rx_modbus[rx_bufSize];
	uint8_t tx_modbus[rx_bufSize];
	uint16_t start_count = 0;
	uint16_t end_count = 0;
	uint8_t flag = 0;
	uint16_t crc = 0;
	int time_out = 25;
  /* Infinite loop */
  for(;;)
  {
  	rx_Count = rx_bufSize - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  	if(flag==1 && rx_Count==end_count)
  	{
  		flag=0;
  		int len=0;
  		for(;start_count!=end_count;)
  		{
  			rx_modbus[len++] = rx_data[start_count++];
  			if(start_count>=rx_bufSize)start_count = 0;
  		}

  		crc=CRC16_Modbus(rx_modbus,len-2);
  		if(BYTE0(crc)==rx_modbus[len-2]&&BYTE1(crc)==rx_modbus[len-1])
  		{
  			time_out=25;
  			if(rx_modbus[1]==0x03)
  			{
					uint16_t tx_len=0;
					uint16_t adrr=0;
					tx_modbus[tx_len++] = rx_modbus[0];
					tx_modbus[tx_len++] = rx_modbus[1];
					adrr = rx_modbus[2]*256+rx_modbus[3];
					tx_modbus[tx_len++] = rx_modbus[5]*2;
					for(int i=0; i<rx_modbus[5];i++,adrr++)
					{
						tx_modbus[tx_len++] = BYTE1(instruct[(adrr)/11][adrr%11]);
						tx_modbus[tx_len++] = BYTE0(instruct[(adrr)/11][adrr%11]);
					}
					crc=CRC16_Modbus(tx_modbus,tx_len);
					tx_modbus[tx_len++] = BYTE0(crc);
					tx_modbus[tx_len++] = BYTE1(crc);

					HAL_UART_Transmit_DMA(&huart2, tx_modbus, tx_len);
  			}
  			else if(rx_modbus[1]==0x04)
  			{
					uint16_t tx_len=0;
					uint16_t adrr=0;
					tx_modbus[tx_len++] = rx_modbus[0];
					tx_modbus[tx_len++] = rx_modbus[1];
					adrr = rx_modbus[2]*256+rx_modbus[3];
					tx_modbus[tx_len++] = rx_modbus[5]*2;
					for(int i=0; i<rx_modbus[5];i++,adrr++)
					{
						tx_modbus[tx_len++] = BYTE1(adc_data[adrr]);
						tx_modbus[tx_len++] = BYTE0(adc_data[adrr]);
					}
					crc=CRC16_Modbus(tx_modbus,tx_len);
					tx_modbus[tx_len++] = BYTE0(crc);
					tx_modbus[tx_len++] = BYTE1(crc);
					HAL_UART_Transmit_DMA(&huart2, tx_modbus, tx_len);
  			}
  			else if(rx_modbus[1]==0x10)
  			{
					uint16_t tx_len=0;
					uint16_t adrr=0;
					adrr = rx_modbus[2]*256+rx_modbus[3];
					for(int i=0;i<rx_modbus[6];i+=2,adrr++)
					{
						instruct[(adrr)/11][adrr%11] = rx_modbus[7+i]*256+rx_modbus[8+i];
					}
					tx_modbus[tx_len++] = rx_modbus[0];
					tx_modbus[tx_len++] = rx_modbus[1];
					tx_modbus[tx_len++] = rx_modbus[2];
					tx_modbus[tx_len++] = rx_modbus[3];
					tx_modbus[tx_len++] = rx_modbus[4];
					tx_modbus[tx_len++] = rx_modbus[5];
					crc=CRC16_Modbus(tx_modbus,tx_len);
					tx_modbus[tx_len++] = BYTE0(crc);
					tx_modbus[tx_len++] = BYTE1(crc);
					HAL_UART_Transmit_DMA(&huart2, tx_modbus, tx_len);
  			}
  			else if(rx_modbus[1]==0x06)
  			{
					uint16_t adrr=0;
					adrr = rx_modbus[2]*256+rx_modbus[3];
					instruct[(adrr)/11][adrr%11] = rx_modbus[4]*256+rx_modbus[5];
					HAL_UART_Transmit_DMA(&huart2, rx_modbus, 8);
  			}

  		}

  	}
  	if(rx_Count!=end_count)//正在接收数据
  	{
  		end_count = rx_Count;
  		flag = 1;
  	}
  	else//空闲
  	{
  		time_out--;
  		if(time_out<0)//通信断开
  		{
  			time_out=0;
  			instruct[0][0]=0;
  		}
  	}




    osDelay(2);
  }
  /* USER CODE END StartModbusTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
