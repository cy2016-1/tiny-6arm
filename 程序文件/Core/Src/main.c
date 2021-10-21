/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data, 8);
  HAL_UART_Receive_DMA(&huart2, rx_data, rx_bufSize);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_OC_Start_IT(&htim1,TIM_CHANNEL_1);


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int get_max(int *arr){
	int max=0;
	for(int i=3;i<9;i++){
		if(arr[i]>max)max=arr[i];
	}
	return max;
}
int max(int a, int b){
	if(a>b)return a;
	else return b;
}

int max_pulse=0;
int step1_err=0;
int step2_err=0;
int step3_err=0;
int step4_err=0;
int step5_err=0;
int step6_err=0;

int step1_foc=0;
int step2_foc=0;
int step3_foc=0;
int step4_foc=0;
int step5_foc=0;
int step6_foc=0;

float step1_scale=0;
float step2_scale=0;
float step3_scale=0;
float step4_scale=0;
float step5_scale=0;
float step6_scale=0;

char step1_flag=0;
char step2_flag=0;
char step3_flag=0;
char step4_flag=0;
char step5_flag=0;
char step6_flag=0;

long cur_num = 0;
int acc_pulse = 0;
int instruct_speed = 0;
char step4_is_run = 0;

double x=0,y=0,z=0,a1=0,a2=0,a3=0;
double L1=2520,L2=1410,L3=0;

double ik_a1=0,ik_a2=0,ik_a3=0,ik_a4=0,ik_a5=0,ik_a6=0;
double new_x=0,new_y=0,new_z=0,ik_x,ik_y,ik_z;
int num =0;
int max_xyz=0;
int last_xyz=0;
char key_flag = 0;
char max_flag = 0;
int start_pul1,start_pul2,start_pul3,start_pul4,start_pul5,start_pul6;
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim==&htim1){

		HAL_GPIO_WritePin(STEP1_PUL_GPIO_Port,STEP1_PUL_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP2_PUL_GPIO_Port,STEP2_PUL_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP3_PUL_GPIO_Port,STEP3_PUL_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP4_PUL_GPIO_Port,STEP4_PUL_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(STEP6_PUL_GPIO_Port,STEP6_PUL_Pin,GPIO_PIN_SET);

//		num++;
//		if(num%10==0)
//		{
//			L3 = sqrt(-2.0*L1*L2*cos((cur_pulse3*0.0113924+180)/57.2958)+L1*L1+L2*L2);
//			float cc = cur_pulse2*0.0067164/57.2958 - acos((L1*L1+L3*L3-L2*L2)/(2.0*L1*L3));
//			z = L3*sin(cc);
//			instruct[4][0] = (int)z;
//			x = cos(cur_pulse1*0.006338/57.2958)*L3*cos(cc);
//			y = sin(cur_pulse1*0.006338/57.2958)*L3*cos(cc);
//			instruct[4][1] = (int)x;
//			instruct[4][2] = (int)y;
//		}

		if(!is_start){
			if(foc_cut[0]>100){
				step1_foc++;
				if(step1_foc>(10000/(foc_cut[0]))){
					step1_foc=0;
					HAL_GPIO_WritePin(STEP1_DIR_GPIO_Port,STEP1_DIR_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(STEP1_PUL_GPIO_Port,STEP1_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse1++;
				}
			}

			if(foc_cut[0]<-100){
				step1_foc--;
				if(step1_foc<(10000/(foc_cut[0]))){
					step1_foc=0;
					HAL_GPIO_WritePin(STEP1_DIR_GPIO_Port,STEP1_DIR_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(STEP1_PUL_GPIO_Port,STEP1_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse1--;
				}
			}





			if(foc_cut[1]>100){
				step2_foc++;
				if(step2_foc>(20000/(foc_cut[1]))){
					step2_foc=0;
					HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port,STEP2_DIR_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(STEP2_PUL_GPIO_Port,STEP2_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse2++;
				}
			}

			if(foc_cut[1]<-100){
				step2_foc--;
				if(step2_foc<(20000/(foc_cut[1]))){
					step2_foc=0;
					HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port,STEP2_DIR_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(STEP2_PUL_GPIO_Port,STEP2_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse2--;
				}
			}




			if(foc_cut[2]>100){
				step3_foc++;
				if(step3_foc>(10000/(foc_cut[2]))){
					step3_foc=0;
					HAL_GPIO_WritePin(STEP3_DIR_GPIO_Port,STEP3_DIR_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(STEP3_PUL_GPIO_Port,STEP3_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse3++;
				}
			}

			if(foc_cut[2]<-100){
				step3_foc--;
				if(step3_foc<(10000/(foc_cut[2]))){
					step3_foc=0;
					HAL_GPIO_WritePin(STEP3_DIR_GPIO_Port,STEP3_DIR_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(STEP3_PUL_GPIO_Port,STEP3_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse3--;
				}
			}




			if(foc_cut[3]>100){
				step4_foc++;
				if(step4_foc>(10000/(foc_cut[3]))){
					step4_foc=0;
					HAL_GPIO_WritePin(STEP4_DIR_GPIO_Port,STEP4_DIR_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(STEP4_PUL_GPIO_Port,STEP4_PUL_Pin,GPIO_PIN_SET);
					cur_pulse4++;
					HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);

				}
			}

			if(foc_cut[3]<-100){
				step4_foc--;
				if(step4_foc<(10000/(foc_cut[3]))){
					step4_foc=0;
					HAL_GPIO_WritePin(STEP4_DIR_GPIO_Port,STEP4_DIR_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(STEP4_PUL_GPIO_Port,STEP4_PUL_Pin,GPIO_PIN_SET);
					cur_pulse4--;
					HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);
				}
			}




			if(foc_cut[4]>100){
				step5_foc++;
				if(step5_foc>(10000/(foc_cut[4]))){
					step5_foc=0;
					HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse5++;
				}
			}

			if(foc_cut[4]<-100){
				step5_foc--;
				if(step5_foc<(10000/(foc_cut[4]))){
					step5_foc=0;
					HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse5--;
				}
			}





			if(foc_cut[5]>100){
				step6_foc++;
				if(step6_foc>(5000/(foc_cut[5]))){
					step6_foc=0;
					HAL_GPIO_WritePin(STEP6_DIR_GPIO_Port,STEP6_DIR_Pin,GPIO_PIN_SET);
					HAL_GPIO_WritePin(STEP6_PUL_GPIO_Port,STEP6_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse6=cur_pulse6+6;
				}
			}

			if(foc_cut[5]<-100){
				step6_foc--;
				if(step6_foc<(5000/(foc_cut[5]))){
					step6_foc=0;
					HAL_GPIO_WritePin(STEP6_DIR_GPIO_Port,STEP6_DIR_Pin,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(STEP6_PUL_GPIO_Port,STEP6_PUL_Pin,GPIO_PIN_RESET);
					cur_pulse6=cur_pulse6-6;
				}
			}


		}









		if((is_start||is_single_run))
		{
			step_time++;
			instruct[0][10] = (int)step_time/100.0;
//			if(max_pulse==(int)step)
			if(((abs(cur_pulse1-instruct[instruct_step][4])<10)&&
				 (abs(cur_pulse2-instruct[instruct_step][5])<10)&&
				 (abs(cur_pulse3-instruct[instruct_step][6])<10))||
				 (key_flag==0&&is_single_run))
			{
				if(!is_single_run&&(int)(step_time/100.0)>instruct[instruct_step][10])
				{
					step_time = 0;
					instruct_step++;instruct[2][2]++;
					key_flag = 0;
				}
				step=0;
				if((instruct[2][2]+10)>instruct[2][1])
				{
					instruct[2][2] = 0;
					instruct_step=10;

					if((instruct[3][4]*65536+instruct[3][3])>=(instruct[3][2]*65536+instruct[3][1]))
					{
						is_start = 0;
						instruct[3][0]=0;
					}
					else{
						cur_num = instruct[3][4]*65536+instruct[3][3];
						cur_num++;
						instruct[3][4] = cur_num>>16;
						instruct[3][3] = cur_num&0xffff;

					}
				}

				max_pulse = max(abs(instruct[instruct_step][4]-cur_pulse1),abs(instruct[instruct_step][5]-cur_pulse2));
				max_pulse = max(max_pulse,abs(instruct[instruct_step][6]-cur_pulse3));
				max_pulse = max(max_pulse,abs(instruct[instruct_step][7]-cur_pulse4));
				max_pulse = max(max_pulse,abs(instruct[instruct_step][8]-cur_pulse5));
				max_pulse = max(max_pulse,abs(instruct[instruct_step][9]-cur_pulse6));
				step1_scale=abs(instruct[instruct_step][4]-cur_pulse1)/(float)max_pulse;
				step2_scale=abs(instruct[instruct_step][5]-cur_pulse2)/(float)max_pulse;
				step3_scale=abs(instruct[instruct_step][6]-cur_pulse3)/(float)max_pulse;
				step4_scale=abs(instruct[instruct_step][7]-cur_pulse4)/(float)max_pulse;
				step5_scale=abs(instruct[instruct_step][8]-cur_pulse5)/(float)max_pulse;
				step6_scale=abs(instruct[instruct_step][9]-cur_pulse6)/(float)max_pulse;

//				if(     abs(instruct[instruct_step][4]-cur_pulse1)==max_pulse)max_flag = 1;
//				else if(abs(instruct[instruct_step][5]-cur_pulse2)==max_pulse)max_flag = 2;
//				else if(abs(instruct[instruct_step][6]-cur_pulse3)==max_pulse)max_flag = 3;
//				else if(abs(instruct[instruct_step][7]-cur_pulse4)==max_pulse)max_flag = 4;
//				else if(abs(instruct[instruct_step][8]-cur_pulse5)==max_pulse)max_flag = 5;
//				else if(abs(instruct[instruct_step][9]-cur_pulse6)==max_pulse)max_flag = 6;

//				step_speed = 0;
//				instruct_speed = (int)instruct[instruct_step][2]*instruct[3][5]/100.0;
			}
			instruct_speed = (int)instruct[instruct_step][2]*instruct[3][5]/100.0;
			if(instruct[instruct_step][0]==0)
			{

				if((key_flag==0&&is_single_run)||(key_flag==0&&is_start))
				{
					step_speed = 0;
					step=0;
					max_pulse = max(abs(instruct[instruct_step][4]-cur_pulse1),abs(instruct[instruct_step][5]-cur_pulse2));
					max_pulse = max(max_pulse,abs(instruct[instruct_step][6]-cur_pulse3));
					max_pulse = max(max_pulse,abs(instruct[instruct_step][7]-cur_pulse4));
					max_pulse = max(max_pulse,abs(instruct[instruct_step][8]-cur_pulse5));
					max_pulse = max(max_pulse,abs(instruct[instruct_step][9]-cur_pulse6));
					step1_scale=abs(instruct[instruct_step][4]-cur_pulse1)/(float)max_pulse;
					step2_scale=abs(instruct[instruct_step][5]-cur_pulse2)/(float)max_pulse;
					step3_scale=abs(instruct[instruct_step][6]-cur_pulse3)/(float)max_pulse;
					step4_scale=abs(instruct[instruct_step][7]-cur_pulse4)/(float)max_pulse;
					step5_scale=abs(instruct[instruct_step][8]-cur_pulse5)/(float)max_pulse;
					step6_scale=abs(instruct[instruct_step][9]-cur_pulse6)/(float)max_pulse;

					key_flag=1;

				}
				instruct[0][2] = (int)step_speed;
				instruct[0][3] = (int)((step_acc)*((instruct_speed)*1.1-step_speed)/instruct_speed);
				if((int)step_speed<(instruct_speed)&&((float)step<((float)max_pulse/2.0)))
				{
					step_acc = instruct[instruct_step][3];
	//				step_speed=step_speed+(step_acc/1000.0)*(1-sin(((instruct[instruct_step][2])-step_speed)/162.4));
					step_speed=step_speed+(step_acc/1000.0)*((instruct_speed)*1.1-step_speed)/instruct_speed;
					acc_pulse = (int)step;
				}

				if((int)step_speed>(instruct_speed)||(((int)step)>(max_pulse-acc_pulse)&&(int)step_speed>2))
				{
					step_acc = instruct[instruct_step][3];
	//				step_speed=step_speed-(step_acc/1000.0)*(1-sin((step_speed-(instruct[instruct_step][2]))/162.4));
					step_speed=step_speed-(step_acc/1000.0)*((instruct_speed)*1.1-step_speed)/instruct_speed;
				}

				step=step+step_speed/255.0;

				if((step1_err!=(int)(step*step1_scale))&&(abs(foc_cut[0])<800)){
					step1_err=(int)(step*step1_scale);
					if(cur_pulse1<instruct[instruct_step][4]){
						HAL_GPIO_WritePin(STEP1_DIR_GPIO_Port,STEP1_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP1_PUL_GPIO_Port,STEP1_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse1++;

					}
					else if (cur_pulse1>instruct[instruct_step][4]){
						HAL_GPIO_WritePin(STEP1_DIR_GPIO_Port,STEP1_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP1_PUL_GPIO_Port,STEP1_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse1--;

					}

				}

				if((step2_err!=(int)(step*step2_scale))&&(abs(foc_cut[1])<800)){
					step2_err=(int)(step*step2_scale);
					if(cur_pulse2<instruct[instruct_step][5]){
						HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port,STEP2_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP2_PUL_GPIO_Port,STEP2_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse2++;

					}
					else if (cur_pulse2>instruct[instruct_step][5]){
						HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port,STEP2_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP2_PUL_GPIO_Port,STEP2_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse2--;

					}

				}

				if((step3_err!=(int)(step*step3_scale))&&(abs(foc_cut[2])<800)){
					step3_err=(int)(step*step3_scale);
					if(cur_pulse3<instruct[instruct_step][6]){
						HAL_GPIO_WritePin(STEP3_DIR_GPIO_Port,STEP3_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP3_PUL_GPIO_Port,STEP3_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse3++;

					}
					else if (cur_pulse3>instruct[instruct_step][6]){
						HAL_GPIO_WritePin(STEP3_DIR_GPIO_Port,STEP3_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP3_PUL_GPIO_Port,STEP3_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse3--;

					}

				}

				if((step4_err!=(int)(step*step4_scale))&&(abs(foc_cut[3])<800)){

					step4_err=(int)(step*step4_scale);
					if(cur_pulse4<instruct[instruct_step][7]){
						step4_is_run = 1;
						HAL_GPIO_WritePin(STEP4_DIR_GPIO_Port,STEP4_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP4_PUL_GPIO_Port,STEP4_PUL_Pin,GPIO_PIN_SET);
						cur_pulse4++;
						HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);

					}
					else if (cur_pulse4>instruct[instruct_step][7]){
						step4_is_run = 1;
						HAL_GPIO_WritePin(STEP4_DIR_GPIO_Port,STEP4_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP4_PUL_GPIO_Port,STEP4_PUL_Pin,GPIO_PIN_SET);
						cur_pulse4--;
						HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);

					}


				}else step4_is_run=0;

				if((step5_err!=(int)(step*step5_scale))&&(step4_is_run==0)&&(abs(foc_cut[4])<800)){
					step5_err=(int)(step*step5_scale);
					if(cur_pulse5<instruct[instruct_step][8]){
						HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse5++;
					}
					else if (cur_pulse5>instruct[instruct_step][8]){
						HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse5--;
					}

				}

				if((step6_err!=(int)(step*step6_scale))&&(abs(foc_cut[5])<800)){
					step6_err=(int)(step*step6_scale);
					if(cur_pulse6<instruct[instruct_step][9]){
						HAL_GPIO_WritePin(STEP6_DIR_GPIO_Port,STEP6_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP6_PUL_GPIO_Port,STEP6_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse6=cur_pulse6+6;
					}
					else if (cur_pulse6>instruct[instruct_step][9]){
						HAL_GPIO_WritePin(STEP6_DIR_GPIO_Port,STEP6_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP6_PUL_GPIO_Port,STEP6_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse6=cur_pulse6-6;
					}
				}


			}

			if(instruct[instruct_step][0]==1)
			{

				if((key_flag==0&&is_single_run)||(key_flag==0&&is_start))
				{
					key_flag = 1;
					start_pul1 = cur_pulse1;
					start_pul2 = cur_pulse2;
					start_pul3 = cur_pulse3;
					start_pul4 = cur_pulse4;
					start_pul5 = cur_pulse5;
					start_pul6 = cur_pulse6;

					L3 = sqrt(-2.0*L1*L2*cos((cur_pulse3*0.0113924+180)/57.2958)+L1*L1+L2*L2);
					float cc = cur_pulse2*0.0067164/57.2958 - acos((L1*L1+L3*L3-L2*L2)/(2.0*L1*L3));
					z = L3*sin(cc);
					x = cos(cur_pulse1*0.006338/57.2958)*L3*cos(cc);
					y = sin(cur_pulse1*0.006338/57.2958)*L3*cos(cc);
					new_x = x;
					new_y = y;
					new_z = z;


					double ik_L3 = sqrt(-2.0*L1*L2*cos((instruct[instruct_step][6]*0.0113924+180)/57.2958)+L1*L1+L2*L2);
					double ccc = instruct[instruct_step][5]*0.0067164/57.2958 - acos((L1*L1+ik_L3*ik_L3-L2*L2)/(2.0*L1*ik_L3));
					ik_z = ik_L3*sin(ccc);
					ik_x = cos(instruct[instruct_step][4]*0.006338/57.2958)*ik_L3*cos(ccc);
					ik_y = sin(instruct[instruct_step][4]*0.006338/57.2958)*ik_L3*cos(ccc);
					max_xyz = max(abs((int)ik_z-(int)z),abs((int)ik_y-(int)y));
					max_xyz = max(max_xyz,abs((int)ik_x-(int)x));
					if(max_xyz>3)
					{
						new_z = new_z+(ik_z-z)*2.0/(float)max_xyz;
						new_x = new_x+(ik_x-x)*2.0/(float)max_xyz;
						new_y = new_y+(ik_y-y)*2.0/(float)max_xyz;
						instruct[4][0] = (int)ik_z;
						instruct[4][1] = (int)ik_x;
						instruct[4][2] = (int)ik_y;

						instruct[4][8] = (int)new_z;
						instruct[4][9] = (int)new_x;
						instruct[4][10] = (int)new_y;

						double new_L3 = sqrt(new_z*new_z+new_x*new_x+new_y*new_y);
						ik_a1 = atan2(new_y,new_x)*57.2958/0.006338;
						ik_a2 = (atan2(new_z,sqrt(new_x*new_x+new_y*new_y))+acos((L1*L1+new_L3*new_L3-L2*L2)/(2*L1*new_L3)))*57.2958/0.0067164;
						ik_a3 = acos((L1*L1-new_L3*new_L3+L2*L2)/(2*L1*L2))*57.2958/0.0113924;
						ik_a4 = -(-90.0/0.0156673 + (atan2(sqrt(new_x*new_x+new_y*new_y),new_z)+
								acos((L2*L2+new_L3*new_L3-L1*L1)/(2*L2*new_L3)))*57.2958/0.0156673)*4.0/90.0;



						instruct[4][3] = (int)(ik_a1*0.006338);
						instruct[4][4] = (int)(ik_a2*0.0067164);
						instruct[4][5] = (int)(ik_a3*0.0113924);
						instruct[4][6] = (int)(ik_a4*0.0156673);

						max_pulse = max(abs((int)ik_a1-cur_pulse1),abs((int)ik_a2-cur_pulse2));
						max_pulse = max(max_pulse,abs((int)(ik_a3-180.0/0.0113924)-cur_pulse3));
	//					max_pulse = max(max_pulse,abs(instruct[instruct_step][7]-cur_pulse4));
	//					max_pulse = max(max_pulse,abs(instruct[instruct_step][8]-cur_pulse5));
	//					max_pulse = max(max_pulse,abs(instruct[instruct_step][9]-cur_pulse6));
						step1_scale=abs((int)ik_a1-cur_pulse1)/(float)max_pulse;
						step2_scale=abs((int)ik_a2-cur_pulse2)/(float)max_pulse;
						step3_scale=abs((int)(ik_a3-180.0/0.0113924)-cur_pulse3)/(float)max_pulse;
//						step4_scale=abs((int)(ik_a4)-cur_pulse4)/(float)max_pulse;
	//					step5_scale=abs(instruct[instruct_step][8]-cur_pulse5)/(float)max_pulse;
	//					step6_scale=abs(instruct[instruct_step][9]-cur_pulse6)/(float)max_pulse;

						step_acc = instruct[instruct_step][3];
						float distance_start =(sqrt(
								(cur_pulse1-start_pul1)*(cur_pulse1-start_pul1)+
								(cur_pulse2-start_pul2)*(cur_pulse2-start_pul2)+
								(cur_pulse3-start_pul3)*(cur_pulse3-start_pul3)));
						float distance_all =(sqrt(
								(start_pul1-instruct[instruct_step][4])*(start_pul1-instruct[instruct_step][4])+
								(start_pul2-instruct[instruct_step][5])*(start_pul2-instruct[instruct_step][5])+
								(start_pul3-instruct[instruct_step][6])*(start_pul3-instruct[instruct_step][6])));
						float speed_start = distance_start*sqrt((step_acc/100.0)*instruct[3][5]/5000.0)+3;
						if(((int)(speed_start)<instruct_speed)&&((int)(distance_start)<(int)(distance_all/2.0)))
						{
							step_speed = speed_start;
						}

						float distance_end =(sqrt(
								(cur_pulse1-instruct[instruct_step][4])*(cur_pulse1-instruct[instruct_step][4])+
								(cur_pulse2-instruct[instruct_step][5])*(cur_pulse2-instruct[instruct_step][5])+
								(cur_pulse3-instruct[instruct_step][6])*(cur_pulse3-instruct[instruct_step][6])));
						float speed_end = distance_end*sqrt((step_acc/100.0)*instruct[3][5]/5000.0)+3;
						if(((int)(speed_end)<instruct_speed)&&((int)(distance_end)<(int)(distance_all/2.0)))
						{
							step_speed = speed_end;
						}


					}
				}


				if((abs(((int)ik_a1)-cur_pulse1)<3)&&
					(abs(((int)ik_a2)-cur_pulse2)<3)&&
					(abs(((int)(ik_a3-180/0.0113924))-cur_pulse3)<3)&&
					((abs((int)ik_z-(int)new_z)>1)||
					(abs((int)ik_x-(int)new_x)>1)||
					(abs((int)ik_y-(int)new_y)>1)))
				{

					new_z = new_z+(ik_z-z)*2.0/(float)max_xyz;
					new_x = new_x+(ik_x-x)*2.0/(float)max_xyz;
					new_y = new_y+(ik_y-y)*2.0/(float)max_xyz;
					instruct[4][0] = (int)ik_z;
					instruct[4][1] = (int)ik_x;
					instruct[4][2] = (int)ik_y;

					instruct[4][8] = (int)new_z;
					instruct[4][9] = (int)new_x;
					instruct[4][10] = (int)new_y;

					double new_L3 = sqrt(new_z*new_z+new_x*new_x+new_y*new_y);
					ik_a1 = atan2(new_y,new_x)*57.2958/0.006338;
					ik_a2 = (atan2(new_z,sqrt(new_x*new_x+new_y*new_y))+acos((L1*L1+new_L3*new_L3-L2*L2)/(2.0*L1*new_L3)))*57.2958/0.0067164;
					ik_a3 = acos((L1*L1-new_L3*new_L3+L2*L2)/(2.0*L1*L2))*57.2958/0.0113924;
					ik_a4 = -(-90.0/0.0156673 + (atan2(sqrt(new_x*new_x+new_y*new_y),new_z)+
							acos((L2*L2+new_L3*new_L3-L1*L1)/(2.0*L2*new_L3)))*57.2958/0.0156673)*4.0/90.0;


					instruct[4][3] = (int)(ik_a1*0.006338);
					instruct[4][4] = (int)(ik_a2*0.0067164);
					instruct[4][5] = (int)(ik_a3*0.0113924);
					instruct[4][6] = (int)(ik_a4*0.0156673);
					instruct[4][7] = (int)new_L3;


					max_pulse = max(abs((int)ik_a1-cur_pulse1),abs((int)ik_a2-cur_pulse2));
					max_pulse = max(max_pulse,abs((int)(ik_a3-180.0/0.0113924)-cur_pulse3));
//					max_pulse = max(max_pulse,abs(instruct[instruct_step][7]-cur_pulse4));
//					max_pulse = max(max_pulse,abs(instruct[instruct_step][8]-cur_pulse5));
//					max_pulse = max(max_pulse,abs(instruct[instruct_step][9]-cur_pulse6));
					step1_scale=abs((int)ik_a1-cur_pulse1)/(float)max_pulse;
					step2_scale=abs((int)ik_a2-cur_pulse2)/(float)max_pulse;
					step3_scale=abs((int)(ik_a3-180.0/0.0113924)-cur_pulse3)/(float)max_pulse;
//					step4_scale=abs((int)(ik_a4)-cur_pulse4)/(float)max_pulse;
//					step4_scale=abs(instruct[instruct_step][7]-cur_pulse4)/(float)max_pulse;
//					step5_scale=abs(instruct[instruct_step][8]-cur_pulse5)/(float)max_pulse;
//					step6_scale=abs(instruct[instruct_step][9]-cur_pulse6)/(float)max_pulse;
					step_acc = instruct[instruct_step][3];
					float distance_start =(sqrt(
							(cur_pulse1-start_pul1)*(cur_pulse1-start_pul1)+
							(cur_pulse2-start_pul2)*(cur_pulse2-start_pul2)+
							(cur_pulse3-start_pul3)*(cur_pulse3-start_pul3)));
					float distance_all =(sqrt(
							(start_pul1-instruct[instruct_step][4])*(start_pul1-instruct[instruct_step][4])+
							(start_pul2-instruct[instruct_step][5])*(start_pul2-instruct[instruct_step][5])+
							(start_pul3-instruct[instruct_step][6])*(start_pul3-instruct[instruct_step][6])));
					float speed_start = distance_start*sqrt((step_acc/100.0)*instruct[3][5]/5000.0)+3;
					if(((int)(speed_start)<instruct_speed)&&((int)(distance_start)<(int)(distance_all/2.0)))
					{
						step_speed = speed_start;
					}

					float distance_end =(sqrt(
							(cur_pulse1-instruct[instruct_step][4])*(cur_pulse1-instruct[instruct_step][4])+
							(cur_pulse2-instruct[instruct_step][5])*(cur_pulse2-instruct[instruct_step][5])+
							(cur_pulse3-instruct[instruct_step][6])*(cur_pulse3-instruct[instruct_step][6])));
					float speed_end = distance_end*sqrt((step_acc/100.0)*instruct[3][5]/5000.0)+3;
					if(((int)(speed_end)<instruct_speed)&&((int)(distance_end)<(int)(distance_all/2.0)))
					{
						step_speed = speed_end;
					}


				}



//				if((abs((int)ik_z-(int)new_z)<1)&&
//						(abs((int)ik_x-(int)new_x)<1)&&
//						(abs((int)ik_y-(int)new_y)<1))
//				{
//					ik_a1 = instruct[instruct_step][4];
//					ik_a2 = instruct[instruct_step][5];
//					ik_a3 = instruct[instruct_step][7];
//
//				}


				instruct[0][2] = (int)step_speed;
				instruct[0][3] = (int)(((step_acc)*instruct[3][5]/100.0));
				step=step+step_speed/255.0;


				if((step1_err!=(int)(step*step1_scale))&&(abs(foc_cut[0])<800)){
					step1_err=(int)(step*step1_scale);
					if(cur_pulse1<(int)ik_a1){
						HAL_GPIO_WritePin(STEP1_DIR_GPIO_Port,STEP1_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP1_PUL_GPIO_Port,STEP1_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse1++;

					}
					else if (cur_pulse1>(int)ik_a1){
						HAL_GPIO_WritePin(STEP1_DIR_GPIO_Port,STEP1_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP1_PUL_GPIO_Port,STEP1_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse1--;

					}

				}

				if((step2_err!=(int)(step*step2_scale))&&(abs(foc_cut[1])<800)){
					step2_err=(int)(step*step2_scale);
					if(cur_pulse2<(int)ik_a2){
						HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port,STEP2_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP2_PUL_GPIO_Port,STEP2_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse2++;

					}
					else if (cur_pulse2>(int)ik_a2){
						HAL_GPIO_WritePin(STEP2_DIR_GPIO_Port,STEP2_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP2_PUL_GPIO_Port,STEP2_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse2--;

					}

				}

				if((step3_err!=(int)(step*step3_scale))&&(abs(foc_cut[2])<800))
				{
					step3_err=(int)(step*step3_scale);
					if(cur_pulse3<(int)(ik_a3-180.0/0.0113924))
					{
						HAL_GPIO_WritePin(STEP3_DIR_GPIO_Port,STEP3_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP3_PUL_GPIO_Port,STEP3_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse3++;

					}
					else if (cur_pulse3>(int)(ik_a3-180.0/0.0113924))
					{
						HAL_GPIO_WritePin(STEP3_DIR_GPIO_Port,STEP3_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP3_PUL_GPIO_Port,STEP3_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse3--;

					}

				}

				if((step4_err!=(int)(step*step4_scale))&&(abs(foc_cut[3])<800))
				{
					step4_err=(int)(step*step4_scale);
//					if(cur_pulse4<(int)(ik_a4))
					if(cur_pulse4<instruct[instruct_step][7])
					{
						step4_is_run = 1;
						HAL_GPIO_WritePin(STEP4_DIR_GPIO_Port,STEP4_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP4_PUL_GPIO_Port,STEP4_PUL_Pin,GPIO_PIN_SET);
						cur_pulse4++;
						HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);

					}
//					else if(cur_pulse4>(int)(ik_a4))
					else if (cur_pulse4>instruct[instruct_step][7])
					{
						step4_is_run = 1;
						HAL_GPIO_WritePin(STEP4_DIR_GPIO_Port,STEP4_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP4_PUL_GPIO_Port,STEP4_PUL_Pin,GPIO_PIN_SET);
						cur_pulse4--;
						HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);

					}


				}else step4_is_run=0;

				if((step5_err!=(int)(step*step5_scale))&&(step4_is_run==0)&&(abs(foc_cut[4])<800)){
					step5_err=(int)(step*step5_scale);
					if(cur_pulse5<instruct[instruct_step][8]){
						HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse5++;
					}
					else if (cur_pulse5>instruct[instruct_step][8]){
						HAL_GPIO_WritePin(STEP5_DIR_GPIO_Port,STEP5_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP5_PUL_GPIO_Port,STEP5_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse5--;
					}

				}

				if((step6_err!=(int)(step*step6_scale))&&(abs(foc_cut[5])<800)){
					step6_err=(int)(step*step6_scale);
					if(cur_pulse6<instruct[instruct_step][9]){
						HAL_GPIO_WritePin(STEP6_DIR_GPIO_Port,STEP6_DIR_Pin,GPIO_PIN_SET);
						HAL_GPIO_WritePin(STEP6_PUL_GPIO_Port,STEP6_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse6=cur_pulse6+6;
					}
					else if (cur_pulse6>instruct[instruct_step][9]){
						HAL_GPIO_WritePin(STEP6_DIR_GPIO_Port,STEP6_DIR_Pin,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(STEP6_PUL_GPIO_Port,STEP6_PUL_Pin,GPIO_PIN_RESET);
						cur_pulse6=cur_pulse6-6;
					}
				}


			}




		}else key_flag = 0;


	}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
