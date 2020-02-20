//Battery 26.4 DCV
//
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Defs.h"
#include "BNO055.h"
#include "SBB_rccTIM.h"

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TESTMODE 		0
#define USE_BNO055 	    1

#define PI 3.1415926535897932384

#define WHEEL_RADIUS 163.5		//Unit : [mm]				//바퀴 지름
#define LENGTH 785					//Unit : [mm]				//바퀴사이 간격

//#define MAX_SPEED_LEFT 	1.810	//Unit : [mm/ms]		//왼쪽 바퀴 최대 속도
#define MAX_SPEED_LEFT 1.860	//Unit : [mm/ms]		//오른쪽 바퀴 최대 속도
#define MAX_SPEED_RIGHT 1.860	//Unit : [mm/ms]		//오른쪽 바퀴 최대 속도
#define MAX_SPEED 1.800

#define DRIVE_SPEED_LIMIT 0.3
			//lSpeed = (WHEEL_RADIUS*2*PI*(lEnc_Cnt - lEnc_Cnt_prev)/ENC_PER_REVOLUTION);//		Unit : [mm/ms]
			

#define LINEAR_ANGULAR_RATE 3		// 1: 3			1대 1 비율이 되면 510
#define MAX_DUTYRATE 100					//최대 PWM 듀티 비

#define MAX_CCR_LEFT 3499						//CCR 최댓값
#define MAX_CCR_RIGHT 6999		

#if TESTMODE==1
	#define JOYSTICK_CALIBRATE_TIME 0		//조이스틱 calibration 횟수, 50 -> 50개의 데이터 샘플 평균
#elif TESTMODE==0
	#define JOYSTICK_CALIBRATE_TIME 30
#endif
	

#define ENC_PER_REVOLUTION 13500			//바퀴 1회전당 엔코더 펄스 수

//PID Gain 
#define P_GAIN_LEFT_CW 	0.25//.8//1.5
#define I_GAIN_LEFT_CW		0//.004
#define D_GAIN_LEFT_CW	0.0008

#define P_GAIN_RIGHT_CW 	0.5//.8//.5
#define I_GAIN_RIGHT_CW		0//.005
#define D_GAIN_RIGHT_CW		0.0008

#define P_GAIN_LEFT_CCW 	0.5//.8//.5
#define I_GAIN_LEFT_CCW		0//.004
#define D_GAIN_LEFT_CCW		0.0008

#define P_GAIN_RIGHT_CCW 	0.25//.8//.5
#define I_GAIN_RIGHT_CCW		0//.005
#define D_GAIN_RIGHT_CCW	0.0008

#define VELOCITY2DUTYGAIN_RIGHT 48.5627//53.55061536
#define VELOCITY2DUTYGAIN_LEFT 49.2432//54.15061536



#define MAX_INTEGRATE 1.0
#define NUMOFRING 50

#define RING_INCREASE(n) (((1+n)>(NUMOFRING-1))? 0 : n+1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//UART 통신을 위한 변수들 

UART_Data UART_Rx;
UART_Data UART5_Tx;
UART_Data UART4_Tx;
UART_Data USART2_Tx;

uint8_t UART_ClearTime_Flg =0;			//오버런 발생 예방을 위한 UART초기화 플래그
uint8_t Joystick_ClearTime_Flg =0;

joystick_data Joystick;

int16_t angular_speed;						//X좌표 값					
int16_t linear_speed;						//Y좌표 값
int8_t isMove=0;			// -1 : Backward , 0 : Stop, 1 : Forward
int8_t prevMove;

int8_t stopState =0;

//Encoder Data
int32_t lEnc_Cnt=0;
int32_t rEnc_Cnt=0;

int32_t lEnc_Cnt_prev=0;
int32_t rEnc_Cnt_prev=0;	

//Speed data
Motor_data motor_R;
Motor_data motor_L;

double dt_t[NUMOFRING];
double dt_a;
int16_t dt_Idx=0,dt_end=1;

int16_t EncR_t[NUMOFRING],EncL_t[NUMOFRING];
int16_t EncL_Idx=0,EncL_end=1;
int16_t EncR_Idx=0,EncR_end=1;
int16_t EncR_a,EncL_a;


//control var 
double heading_Now;
double heading_Goal;
double heading_Err;

double AngularSpeed_Goal;
double AngularSpeed_Now;
double AngularSpeed_Sensor;
double AngularX,AngularY;
double AngularSpeed_Err;


double AngleGoal=0;
double AngleSensor;
double AngleErr;

euler_Vector angles;
euler_Vector eulerAngle;
int8_t dataFlg =0;
int8_t testtmp=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM10_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/**
  * @brief  map함수 8비트 연산, s0~e0범위의 값 value를 s1~e1범위의 값으로 바꿈
  * @param  value : 변환할 값
  * @param  s0 : value값의 최솟값
  * @param  e0 : value값의 최댓값
  * @param  s1 : 변환할 범위의 최솟값
  * @param  e1 : 변환할 범위의 최댓값
  * @retval 변환된 값
  */
int8_t map8(int16_t value,int16_t s0,int16_t e0,int16_t s1,int16_t e1)
{
	int8_t ret= s1 + (value-s0)*((e1-s1)/(1.0*(e0-s0)));
	return ret;
}
/**
  * @brief  map함수 16비트 연산, s0~e0범위의 값 value를 s1~e1범위의 값으로 바꿈
  * @param  value : 변환할 값
  * @param  s0 : value값의 최솟값
  * @param  e0 : value값의 최댓값
  * @param  s1 : 변환할 범위의 최솟값
  * @param  e1 : 변환할 범위의 최댓값
  * @retval 변환된 값
  */
int16_t map16(int16_t value,int16_t s0,int16_t e0,int16_t s1,int16_t e1)
{
	int16_t ret= s1 + (value-s0)*((e1-s1)/(1.0*(e0-s0)));
	if(ret<=s1) ret = s1;
	else if (ret>=e1) ret = e1;
	return ret;
}

/**
  * @brief  map함수 float 연산, s0~e0범위의 값 value를 s1~e1범위의 값으로 바꿈
  * @param  value : 변환할 값		정수
  * @param  s0 : value값의 최솟값	정수
  * @param  e0 : value값의 최댓값	정수
  * @param  s1 : 변환할 범위의 최솟값
  * @param  e1 : 변환할 범위의 최댓값
  * @retval 변환된 값
  */

int sgn(float x){
	return (x<0 ? -1 : 1);
}
float mapf(int16_t value,int16_t s0,int16_t e0,float s1,float e1)
{
	float ret= s1 + (value-s0)*((e1-s1)/(1.0*(e0-s0)));
	if(ret<=s1) ret = s1;
	else if (ret>=e1) ret = e1;
	return ret;
}
/**
  * @brief  BT통신으로 115200bps로 로그를 기록하는 함수 
  * @param  str : 문자열의 시작 주소
  * @retval none
  */
int16_t mapf2(float value,float s0,float e0,float s1,float e1)
{
	int16_t ret= s1 + (value-s0)*((e1-s1)/(1.0*(e0-s0)));
	if(ret<= s1) ret = s1;
	else if (ret>=e1) ret = e1;
	return ret;
}
/**
  * @brief  BT통신으로 115200bps로 로그를 기록하는 함수 
  * @param  str : 문자열의 시작 주소
  * @retval none
  */
void BT_LOG(){		//str은 항상 UART_Txbuf
	if(UART4_Tx.byte.flg) {
		UART4_Tx.idx =0;
		UART4_Tx.dataLength = strlen(UART4_Tx.buf);
		HAL_UART_Transmit_IT(&huart4, (uint8_t *)(UART4_Tx.buf+UART4_Tx.idx++),1);	//
		UART4_Tx.byte.flg =0;
	}
}
void Send_UART4_data(uint8_t* data, uint8_t length)		//
{
	if(UART4_Tx.byte.flg) {
		memcpy(UART4_Tx.buf,data,length);
		UART4_Tx.idx =0;
		UART4_Tx.dataLength = length;
		HAL_UART_Transmit_IT(&huart4, (uint8_t *)(UART4_Tx.buf+UART4_Tx.idx++),1);	//
		UART4_Tx.byte.flg =0;
	}
}
void Send_UART5(char *str)		//
{
	if(UART5_Tx.byte.flg) {
		UART5_Tx.idx =0;
		UART5_Tx.dataLength = strlen(UART5_Tx.buf);
		HAL_UART_Transmit_IT(&huart5, (uint8_t *)(UART5_Tx.buf+UART5_Tx.idx++),1);	//
		UART5_Tx.byte.flg =0;
	}
}
void Send_UART5_data(uint8_t* data, uint8_t length)		//
{
	if(UART5_Tx.byte.flg) {
		memcpy(UART5_Tx.buf,data,length);
		UART5_Tx.idx =0;
		UART5_Tx.dataLength = length;
		HAL_UART_Transmit_IT(&huart5, (uint8_t *)(UART5_Tx.buf+UART5_Tx.idx++),1);	//
		UART5_Tx.byte.flg =0;
	}
}
void Send_USART2()		//
{
	if(USART2_Tx.byte.flg) {
		USART2_Tx.idx =0;
		USART2_Tx.dataLength = strlen(USART2_Tx.buf);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *)(USART2_Tx.buf+USART2_Tx.idx++),1);	//
		USART2_Tx.byte.flg =0;
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4){
		if(UART4_Tx.idx >= UART4_Tx.dataLength){
			UART4_Tx.byte.flg = 1;
		}
		else{
				HAL_UART_Transmit_IT(huart, (uint8_t *)(UART4_Tx.buf+UART4_Tx.idx++),1);	//
		}
	}
	else if(huart->Instance == UART5){
		if(UART5_Tx.idx >= UART5_Tx.dataLength){
			UART5_Tx.byte.flg = 1;
		}
		else{
				HAL_UART_Transmit_IT(huart, (uint8_t *)(UART5_Tx.buf+UART5_Tx.idx++),1);	//
		}
	}
	else if(huart->Instance == USART2){
		if(USART2_Tx.idx >= USART2_Tx.dataLength){
			USART2_Tx.byte.flg = 1;
		}
		else{
				HAL_UART_Transmit_IT(huart, (uint8_t *)(USART2_Tx.buf+USART2_Tx.idx++),1);	//
		}	
	}
	
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		UART_Rx.buf[UART_Rx.idx++] = UART_Rx.byte.data;
		if(UART_Rx.byte.data=='<') {
			Joystick.ReceiveFlg=0;
			UART_Rx.idx=1;
		}
		else if(UART_Rx.byte.data=='>'){
			UART_Rx.buf[UART_Rx.idx] = '\0';
			UART_Rx.idx =0;
			Joystick.ReceiveFlg = 1;
		}
		//huart4.Instance->SR &= ~(HAL_UART_ERROR_ORE);
		UART_ClearTime_Flg = 0;
		HAL_UART_Receive_IT(&huart4,&(UART_Rx.byte.data),1);
	} 
	
	if(huart->Instance == UART5) 
	{
		bno055.Uart5.buf[bno055.Uart5.idx++] = bno055.Uart5.byte.data;
		
		if(bno055.Uart5.buf[0] ==0xBB){		//읽기 성공 스타트 바이트
			if(bno055.Uart5.idx<2);
			else if(bno055.Uart5.idx >= (bno055.Uart5.buf[1]+2)){
				bno055.Uart5.dataLength = bno055.Uart5.idx;
				bno055.Uart5.idx=0;
				bno055.Uart5.RxFlg =1;
				bno055.State = BNO_Ready;
				if(bno055.VectorState == BNO_Vector){		//벡터 읽기 완료
						bno055.VectorState = BNO_Vector_Comp;
				}
			}				
		}
		else if(bno055.Uart5.buf[0] == 0xEE)	//Error Occure
		{
			bno055.State = BNO_Error;
			if(bno055.Uart5.idx>=2) {
				bno055.Uart5.dataLength = bno055.Uart5.idx;
				bno055.Uart5.idx =0;
				bno055.Uart5.RxFlg =1;
			}
		}
		else{
			bno055.Uart5.idx =0;
		}
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED4_Pin);
		UART_ClearTime_Flg = 0;
		HAL_UART_Receive_IT(&huart5,&bno055.Uart5.byte.data,1);
	}
}


/**
  * @brief  DC모터의 속도를 PID제어
  * @param  none
  * @retval none
  */
void DC_PID_Speed_Ctrl(){	
	//if(motor_R.speed_Goal>-MAX_SPEED*0.3 && motor_R.speed_Goal<MAX_SPEED*0.3) motor_R.speed_Goal= 0;
	//if(motor_L.speed_Goal>-MAX_SPEED*0.3 && motor_L.speed_Goal<MAX_SPEED*0.3) motor_L.speed_Goal= 0;
	
	/*	// 가속만 
	if(motor_R.speed_Goal==0) motor_R.speed_Ctrl = 0;
	else if(motor_R.speed_Ctrl < motor_R.speed_Goal) motor_R.speed_Ctrl += R_dt;
	else if(motor_R.speed_Ctrl > motor_R.speed_Goal)  motor_R.speed_Ctrl -= R_dt;
	
	if(motor_L.speed_Goal==0) motor_L.speed_Ctrl = 0;
	else if(motor_L.speed_Ctrl < motor_L.speed_Goal) motor_L.speed_Ctrl += L_dt;
	else if(motor_L.speed_Ctrl > motor_L.speed_Goal) motor_L.speed_Ctrl -= L_dt;
	*/
	
	
	//가,감속
	if(fabs(motor_R.speed_Ctrl -motor_R.speed_Goal)< R_dt) motor_R.speed_Ctrl = motor_R.speed_Goal;
	else if(motor_R.speed_Ctrl < motor_R.speed_Goal) motor_R.speed_Ctrl += R_dt;
	else if(motor_R.speed_Ctrl > motor_R.speed_Goal)  motor_R.speed_Ctrl -= R_dt;
	
	if(fabs(motor_L.speed_Ctrl -motor_L.speed_Goal)< L_dt) motor_L.speed_Ctrl = motor_L.speed_Goal;
	else if(motor_L.speed_Ctrl < motor_L.speed_Goal) motor_L.speed_Ctrl += L_dt;
	else if(motor_L.speed_Ctrl > motor_L.speed_Goal) motor_L.speed_Ctrl -= L_dt;
	
	
		/*
	if(motor_R.speed_Ctrl < motor_R.speed_Goal) motor_R.speed_Ctrl += R_dt;
	else if(motor_R.speed_Ctrl > motor_R.speed_Goal)  motor_R.speed_Ctrl -= R_dt;
	else if(abs(motor_R.speed_Ctrl -motor_R.speed_Goal)< R_dt) motor_R.speed_Ctrl = motor_R.speed_Goal;
	
	if(motor_L.speed_Ctrl < motor_L.speed_Goal) motor_L.speed_Ctrl += L_dt;
	else if(motor_L.speed_Ctrl > motor_L.speed_Goal) motor_L.speed_Ctrl -= L_dt;
	else if(abs(motor_L.speed_Ctrl -motor_L.speed_Goal)< L_dt) motor_L.speed_Ctrl = motor_L.speed_Goal;
		*/
		
		
	AngleGoal += 180*0.001*(motor_R.speed_Ctrl - motor_L.speed_Ctrl) / (LENGTH/1000.0)/PI;
	if(AngleGoal >180) AngleGoal -=360;
	else if(AngleGoal <-180) AngleGoal +=360;
	
	
	//AngularSpeed_Goal= (2000*(motor_R.speed_Ctrl - motor_L.speed_Ctrl)/LENGTH)*180/PI;
	//AngularSpeed_Err = (AngularSpeed_Goal - AngularSpeed_Sensor)/10;
	//AngularSpeed_Err =0;3
	//허용 각도오차 0.2deg
	AngleErr = AngleGoal + AngleSensor;
	if(AngleErr > 180)  AngleErr -=360;
	else if(AngleErr<-180) AngleErr +=360;

	//(fabs(AngleErr)>20 ? sgn(AngleErr)*20 : AngleErr);
	//AngleErr=0;
	
	motor_R.speed_Err = (motor_R.speed_Ctrl+(fabs(AngleErr)>20 ? sgn(AngleErr)*20 : AngleErr)/10.0- motor_R.speed);
	motor_L.speed_Err = (motor_L.speed_Ctrl-(fabs(AngleErr)>20 ? sgn(AngleErr)*20 : AngleErr)/10.0 - motor_L.speed) ;
	
	if(motor_R.speed_Goal>=0) {
		motor_R.speed_pid_data[PID_P] = motor_R.speed_Err* P_GAIN_RIGHT_CW;
		motor_R.speed_pid_data[PID_I] += motor_R.speed_Err*I_GAIN_RIGHT_CW;
		motor_R.speed_pid_data[PID_D] = motor_R.speed_Err*D_GAIN_RIGHT_CW;
	}
	else{
		motor_R.speed_pid_data[PID_P] = motor_R.speed_Err* P_GAIN_RIGHT_CCW;
		motor_R.speed_pid_data[PID_I] += motor_R.speed_Err*I_GAIN_RIGHT_CCW;
		motor_R.speed_pid_data[PID_D] = motor_R.speed_Err*D_GAIN_RIGHT_CCW;
	}
	
	if(motor_L.speed_Goal>=0){
		motor_L.speed_pid_data[PID_P] = motor_L.speed_Err* P_GAIN_LEFT_CW;
		motor_L.speed_pid_data[PID_I] += motor_L.speed_Err*I_GAIN_LEFT_CW;
		motor_L.speed_pid_data[PID_D] = motor_L.speed_Err*D_GAIN_LEFT_CW;
	}
	else{
		motor_L.speed_pid_data[PID_P] = motor_L.speed_Err* P_GAIN_LEFT_CCW;
		motor_L.speed_pid_data[PID_I] += motor_L.speed_Err*I_GAIN_LEFT_CCW;
		motor_L.speed_pid_data[PID_D] = motor_L.speed_Err*D_GAIN_LEFT_CCW;
	}
	
	//Feedforward using TimeTable
	motor_R.speed_pid_data[PID_DATA] = (MAX_SPEED*(motor_R.speed_Ctrl*VELOCITY2DUTYGAIN_RIGHT)/100.0 ) + motor_R.speed_pid_data[PID_P] + motor_R.speed_pid_data[PID_D];
	motor_L.speed_pid_data[PID_DATA] = (MAX_SPEED*(motor_L.speed_Ctrl*VELOCITY2DUTYGAIN_LEFT)/100.0 ) + motor_L.speed_pid_data[PID_P] + motor_L.speed_pid_data[PID_D];
	
	

	//Feedforward constant 1.0
	//motor_R.speed_pid_data[PID_DATA] = motor_R.speed_Ctrl + motor_R.speed_pid_data[PID_P] + motor_R.speed_pid_data[PID_D];
	//motor_L.speed_pid_data[PID_DATA] = motor_L.speed_Ctrl + motor_L.speed_pid_data[PID_P] + motor_L.speed_pid_data[PID_D];
	
	//motor_R.speed_pid_data[PID_DATA] = motor_R.speed_pid_data[PID_P] + motor_R.speed_pid_data[PID_D];
	//motor_L.speed_pid_data[PID_DATA] = motor_L.speed_pid_data[PID_P] + motor_L.speed_pid_data[PID_D];
	
	
	
	
	if(TESTMODE){
		motor_R.motor_ccr = MAX_CCR_RIGHT*motor_R.speed_Goal/100.0;
		motor_L.motor_ccr =  MAX_CCR_LEFT*motor_L.speed_Goal/100.0;
	}else{
		motor_R.motor_ccr = mapf2(motor_R.speed_pid_data[PID_DATA],-MAX_SPEED,MAX_SPEED,-(MAX_CCR_RIGHT*MAX_DUTYRATE/100),(MAX_CCR_RIGHT*MAX_DUTYRATE/100));
		motor_L.motor_ccr = mapf2(motor_L.speed_pid_data[PID_DATA],-MAX_SPEED,MAX_SPEED,-(MAX_CCR_LEFT*MAX_DUTYRATE/100),(MAX_CCR_LEFT*MAX_DUTYRATE/100));		
	}
	
	HAL_GPIO_WritePin(nSLEEP_M1_GPIO_Port,nSLEEP_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(nSLEEP_M2_GPIO_Port,nSLEEP_M2_Pin,GPIO_PIN_SET);	
	
	if(motor_L.motor_ccr<0)
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_SET);
	 else if(motor_L.motor_ccr>0)
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_RESET);
	 else 
		;//HAL_GPIO_WritePin(nSLEEP_M2_GPIO_Port,nSLEEP_M2_Pin,GPIO_PIN_RESET);	
	 
	if(motor_R.motor_ccr<0)
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_RESET);
	 else if(motor_R.motor_ccr>0)
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_SET);
	 else 
		;//HAL_GPIO_WritePin(nSLEEP_M1_GPIO_Port,nSLEEP_M1_Pin,GPIO_PIN_RESET);
			
	TIM4->CCR3 =abs(motor_L.motor_ccr);
	TIM8->CCR4=abs(motor_R.motor_ccr);
}



/**
  * @brief  DC모터의 위치를 PID제어
  * @param  none
  * @retval none
  */
void DC_PID_Position_Ctrl(){	
	//if(motor_R.speed_Goal>-MAX_SPEED_RIGHT*0.01 && motor_R.speed_Goal<MAX_SPEED_RIGHT*0.01) motor_R.speed_Goal= 0;
	//if(motor_L.speed_Goal>-MAX_SPEED_LEFT*0.01 && motor_L.speed_Goal<MAX_SPEED_LEFT*0.01) motor_L.speed_Goal= 0;
	motor_R.position = rEnc_Cnt;
	motor_L.position = lEnc_Cnt;
	
	motor_R.position_Err = (motor_R.position_Goal - motor_R.position);
	motor_L.position_Err = (motor_L.position_Goal - motor_L.position);
	
	if(motor_R.position_pid_data[PID_I] >=MAX_INTEGRATE) motor_R.position_pid_data[PID_I] = MAX_INTEGRATE;
	if(motor_R.position_pid_data[PID_I] <=-MAX_INTEGRATE) motor_R.position_pid_data[PID_I] = -MAX_INTEGRATE;

	if(motor_L.position_pid_data[PID_I] >=MAX_INTEGRATE) motor_L.position_pid_data[PID_I] = MAX_INTEGRATE;
	if(motor_L.position_pid_data[PID_I] <=-MAX_INTEGRATE) motor_L.position_pid_data[PID_I] = -MAX_INTEGRATE;
	
	motor_R.position_pid_data[PID_DATA] = motor_R.position_pid_data[PID_P] + motor_R.position_pid_data[PID_I] + motor_R.position_pid_data[PID_D];
	motor_L.position_pid_data[PID_DATA] = motor_L.position_pid_data[PID_P] + motor_L.position_pid_data[PID_I] + motor_L.position_pid_data[PID_D];
	
	motor_R.motor_ccr = mapf2(motor_R.position_pid_data[PID_DATA],-MAX_SPEED_RIGHT,MAX_SPEED_RIGHT,-(MAX_CCR_RIGHT*MAX_DUTYRATE/100),(MAX_CCR_RIGHT*MAX_DUTYRATE/100));
	motor_L.motor_ccr =  mapf2(motor_L.position_pid_data[PID_DATA],-MAX_SPEED_LEFT,MAX_SPEED_LEFT,-(MAX_CCR_LEFT*MAX_DUTYRATE/100),(MAX_CCR_LEFT*MAX_DUTYRATE/100));
	
	
	HAL_GPIO_WritePin(nSLEEP_M1_GPIO_Port,nSLEEP_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(nSLEEP_M2_GPIO_Port,nSLEEP_M2_Pin,GPIO_PIN_SET);	
	
	if(motor_L.motor_ccr<0)
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_SET);
	 else if(motor_L.motor_ccr>0)
		HAL_GPIO_WritePin(DIR_M2_GPIO_Port,DIR_M2_Pin,GPIO_PIN_RESET);
	 else 
		;//HAL_GPIO_WritePin(nSLEEP_M2_GPIO_Port,nSLEEP_M2_Pin,GPIO_PIN_RESET);	
	 
	if(motor_R.motor_ccr<0)
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_RESET);
	 else if(motor_R.motor_ccr>0)
		HAL_GPIO_WritePin(DIR_M1_GPIO_Port,DIR_M1_Pin,GPIO_PIN_SET);
	 else 
		;//HAL_GPIO_WritePin(nSLEEP_M1_GPIO_Port,nSLEEP_M1_Pin,GPIO_PIN_RESET);
	 
	TIM4->CCR3 =abs(motor_L.motor_ccr);
	TIM8->CCR4=abs(motor_R.motor_ccr);
}

/**
  * @brief  선속도와 각속도로 부터 양 모터의 속도를 계산
  * @param  LeftGoal : 왼쪽 바퀴의 목표 속도
  * @param  RightGoal : 오른쪽 바퀴의 목표 속도
  * @param  angularSpeed : 로봇의 회전 각속도
  * @param  linearSpeed : 로봇의 진행 속도
  * @retval none
  */
void calc_Goal_Speed(float* RightGoal,float* LeftGoal, int16_t angularSpeed,int16_t linearSpeed)
{
	//float angularV = mapf(angluarSpeed,-joystick_CenterX,joystick_CenterX,-510.0,510.0);
	//Joystick 값에 따른 각속도와 선속도를 백분율로 표시
	int16_t max_X = ((Joystick.CenterX < 512-Joystick.CenterX) ? Joystick.CenterX : 512-Joystick.CenterX);
	int16_t max_Y = ((Joystick.CenterY < 512-Joystick.CenterY) ? Joystick.CenterY : 512-Joystick.CenterY);
	
	float velocity = mapf(linearSpeed,-max_Y,max_Y,-100.0,100.0);
	float angularV=mapf(-1*angularSpeed,-max_X,max_X,-100.0,100.0);
									//-1을 곱해 각속도가 양수일때 시계 반대방향이 되도록 
	
	//백분율 속도를 절대 속도로 바꿈 
	
	float NewGoalR;
	float NewGoalL;
	
	if(isMove< 2) {	//명령 모드가 자동(-1~1)일때 선속도 고정 30%
		velocity = isMove*MAX_SPEED*DRIVE_SPEED_LIMIT;
		angularV = (((2*MAX_SPEED*DRIVE_SPEED_LIMIT)/(LENGTH/1000.0))*angularV )/100.0;
	}
	else if(isMove==4){	//직진
		velocity =sgn(linearSpeed)*MAX_SPEED*DRIVE_SPEED_LIMIT;
		angularV = 0;
	}	
	else if(isMove ==3){	//정지
		velocity =0;
		angularV =0;
	}
	else if(isMove==2){		// 자유 조종
		velocity = (MAX_SPEED*DRIVE_SPEED_LIMIT)*velocity /100.0;
		angularV = (((2*MAX_SPEED*DRIVE_SPEED_LIMIT)/(LENGTH/1000.0))*angularV )/100.0;	
	}
	else {
		velocity =0;
		angularV =0;
	}
	NewGoalR = velocity+(angularV*(LENGTH/1000.0)/4.0);		
	NewGoalL = velocity-(angularV*(LENGTH/1000.0)/4.0);
	
	if(fabs(NewGoalR) <=R_dt) NewGoalR =0;
	if(fabs(NewGoalL) <= L_dt) NewGoalL =0;
	
	#if TESTMODE==1
	*RightGoal = angularSpeed;		// unit : [mm/ms]	
	*LeftGoal = linearSpeed;
	#elif TESTMOE ==0
	*RightGoal = NewGoalR;		// unit : [mm/ms]
	*LeftGoal = NewGoalL;
	#endif
}


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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  //MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	int ltimFlg_1000ms    = 0;
	int ltimFlg_100ms    = 0;
	int ltimFlg_10ms	 = 0;
	int joystick_cnt=0;
	
	int16_t nowR,nowL;
	
	HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_UART_Receive_IT(&huart4,&UART_Rx.byte.data,1);
	HAL_UART_Receive_IT(&huart5,&bno055.Uart5.byte.data,1);
	
	
	HAL_TIM_PWM_Init(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Init(&htim8);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);	
	
	HAL_GPIO_WritePin(nSLEEP_M1_GPIO_Port,nSLEEP_M1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(nSLEEP_M2_GPIO_Port,nSLEEP_M2_Pin,GPIO_PIN_SET);	
	
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);

	HAL_GPIO_WritePin(LED3_GPIO_Port,LED3_Pin|LED5_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port,LED4_Pin,GPIO_PIN_SET);
	
	UART4_Tx.byte.flg=1;
	UART5_Tx.byte.flg = 1;
	USART2_Tx.byte.flg = 1;
	
	angles.roll = 0;
	angles.pitch =0;
	angles.yaw =0;
	
	if(USE_BNO055) begin_BNO055();
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(!timFlg_10ms_Prg)
		{
			continue;
		}
		ltimFlg_10ms		=	timFlg_10ms;
		ltimFlg_100ms 	= timFlg_100ms;
		ltimFlg_1000ms 	= timFlg_1000ms;
		
		if(ltimFlg_100ms){		//데이터 출력
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED5_Pin);
			if(USART2_Tx.byte.flg){
				if(TESTMODE) sprintf(USART2_Tx.buf,"%d,%.5f,%d,%.5f\n",motor_R.motor_ccr,motor_R.speed,motor_L.motor_ccr,motor_L.speed);			//Speed Table 
				else{
					//sprintf(USART2_Tx.buf,"%.5f,%.5f,%.5f,%.3f,%.3f,\n",AngularSpeed_Now,AngularSpeed_Goal,AngularSpeed_Sensor,motor_R.speed,motor_L.speed);
					//sprintf(USART2_Tx.buf,"%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",motor_R.speed,motor_R.speed_Goal,motor_L.speed,motor_L.speed_Goal,AngularSpeed_Goal,AngularSpeed_Err);
					//sprintf(USART2_Tx.buf,"%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",motor_R.speed,motor_R.speed_Ctrl,motor_R.speed_Err,motor_L.speed,motor_L.speed_Ctrl,motor_L.speed_Err);
					
					//Matlab
					//sprintf(USART2_Tx.buf,"%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.2f\n",motor_R.speed,motor_R.speed_Ctrl,motor_L.speed,motor_L.speed_Ctrl,AngularSpeed_Sensor,AngularSpeed_Now,AngularX,AngularY,AngularSpeed_Err);
					sprintf(USART2_Tx.buf,"%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f\n",motor_R.speed,motor_R.speed_Ctrl,motor_L.speed,motor_L.speed_Ctrl,AngleGoal,AngleSensor,AngleErr);
					
					//sprintf(USART2_Tx.buf,"%d,%d,%d,%.3f\n",nowR,nowL,EncR_a,dt_a);			//Encoder
					//sprintf(USART2_Tx.buf,"*%.2f,%.2f,%.2f\r\n",eulerAngle.roll,eulerAngle.pitch,eulerAngle.yaw);	//Gyro 
					//sprintf(USART2_Tx.buf,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",angles.roll,angles.pitch,angles.yaw,eulerAngle.roll,eulerAngle.pitch,eulerAngle.yaw);	//euler Angles
					
				}
				Send_USART2();
			}
			if(Joystick_ClearTime_Flg==0) Joystick_ClearTime_Flg =1;
			else {
				Joystick_ClearTime_Flg = 1;
				motor_R.motor_ccr =0;
				motor_L.motor_ccr= 0;
				motor_R.speed_Goal =0;
				motor_L.speed_Goal =0;
				AngleGoal = -AngleSensor;
				AngleErr =0;
				
			}
		}
		
		if(ltimFlg_10ms){
			//if(UART4_TxFlg)sprintf(UART4_Txbuf,"%.3f,%.3f,<%.3f,%.3f>,[%d,%d],{%d,%d}\r\n" ,rSpeed,lSpeed,rSpeed_Goal,lSpeed_Goal,angular_speed,linear_speed,rEnc_Cnt,lEnc_Cnt);
			//SBB_LOG();
			if(USE_BNO055){
				if(bno055.Uart5.RxFlg){		//벡터 읽기
					bno055.Uart5.RxFlg=0;
					if(bno055.State != BNO_Error){
						if(dataFlg ==0){
							calcVector(VECTOR_GYROSCOPE);
							dataFlg = 1;
							AngularSpeed_Sensor = bno055.vec_Data.vector.z;
							AngularX = bno055.vec_Data.vector.x;
							AngularY = bno055.vec_Data.vector.y;
							angles.roll += AngularX * 0.01;
							angles.pitch += AngularY * 0.01;
							angles.yaw += AngularSpeed_Sensor * 0.01;
						}else{
							calcVector(VECTOR_EULER);
							dataFlg =1;
							AngularSpeed_Sensor = (bno055.vec_Data.vector.x > 180 ? bno055.vec_Data.vector.x-360 : bno055.vec_Data.vector.x); 
							AngularX = bno055.vec_Data.vector.z;
							AngularY = bno055.vec_Data.vector.y;
							eulerAngle.roll = AngularX;
							eulerAngle.pitch= AngularY;
							eulerAngle.yaw = AngularSpeed_Sensor;
							AngleSensor = eulerAngle.yaw;
						}
						//getGyroZ();
					}
					
					HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
				}
				if(UART5_Tx.byte.flg){			//	벡터 쓰기
					if(dataFlg==0)
						requestVector(VECTOR_GYROSCOPE);
					else 
						requestVector(VECTOR_EULER);

					//requestGyroZ();
					UART5_Tx.byte.flg=1;
				}
			}
		}
		if(timFlg_20ms_mid){	//UART 초기화
			if(UART_ClearTime_Flg>=5){
				while(!(UART4_Tx.byte.flg));
					MX_UART4_Init();
					UART4_Tx.byte.flg = 1;
					HAL_UART_Receive_IT(&huart4,&UART_Rx.byte.data,1);
				
				while(!(UART5_Tx.byte.flg));
					MX_UART5_Init();
					UART5_Tx.byte.flg = 1;
					HAL_UART_Receive_IT(&huart5,&bno055.Uart5.byte.data,1);
				
			}else UART_ClearTime_Flg = 5;
			
			timFlg_20ms_mid=0;
		}
		
		if(timFlg_1ms){
			rEnc_Cnt= TIM3->CNT;
			lEnc_Cnt = TIM1->CNT;
			
			EncR_t[EncR_Idx] = rEnc_Cnt - rEnc_Cnt_prev;
			EncL_t[EncL_Idx] = lEnc_Cnt - lEnc_Cnt_prev;
			nowR = EncR_t[EncR_Idx] ;
			nowL = EncL_t[EncL_Idx] ;
			//TIM3->CNT =65535/2;
			//TIM1->CNT =65535/2;
			
			rEnc_Cnt_prev=TIM3->CNT;	
			lEnc_Cnt_prev=TIM1->CNT;
			
			EncR_a = (EncR_a - EncR_t[EncR_end] + EncR_t[EncR_Idx]);
			EncL_a = (EncL_a - EncL_t[EncL_end] + EncL_t[EncL_Idx]);
			
			dt_t[dt_Idx] = timCnt_1ms;///1000.0;
			timCnt_1ms =0;
			dt_a = (dt_a - dt_t[dt_end] + dt_t[dt_Idx]);
			
			//모터 속도 측정
			//motor_R.speed = (WHEEL_RADIUS*2*PI*(rEnc_Cnt - rEnc_Cnt_prev)/ENC_PER_REVOLUTION)/msTim_dt;//		Unit : [mm/ms]
			//motor_L.speed  = (WHEEL_RADIUS*2*PI*(lEnc_Cnt - lEnc_Cnt_prev)/ENC_PER_REVOLUTION)/msTim_dt;//		Unit : [mm/ms]
			
			motor_R.speed = (WHEEL_RADIUS*2*PI*(EncR_a)/ENC_PER_REVOLUTION)/dt_a;//		Unit : [mm/ms]
			motor_L.speed  = (WHEEL_RADIUS*2*PI*(EncL_a)/ENC_PER_REVOLUTION)/dt_a;//		Unit : [mm/ms]
			
			//엔코더 값 초기화
			//if(abs(rEnc_Cnt - rEnc_Cnt_prev) >= 30000) TIM3->CNT =65535/2;
			//if(abs(lEnc_Cnt - lEnc_Cnt_prev) >= 30000) TIM1->CNT =65535/2;
			
			EncR_Idx= RING_INCREASE(EncR_Idx);
			EncR_end= RING_INCREASE(EncR_end);
			EncL_Idx= RING_INCREASE(EncL_Idx);
			EncL_end= RING_INCREASE(EncL_end);
			dt_Idx= RING_INCREASE(dt_Idx);
			dt_end= RING_INCREASE(dt_end);
			
			AngularSpeed_Now= (1000*(motor_R.speed- motor_L.speed)/LENGTH)*180/PI;
			
			//AngleGoal +=AngularSpeed_Now *0.001;
			
			//정지된 바퀴 위치 제어  --> 노면에서 미끄러지지 않게
			if(motor_R.speed_Goal ==0 || motor_L.speed_Goal ==0)
				DC_PID_Position_Ctrl();
			
			//속도 제어 
			DC_PID_Speed_Ctrl();
			
			
			timFlg_1ms =0;
		}
		if(timFlg_3ms){
			
			
			timFlg_3ms =0;
		}
		if(timFlg_5ms_BTSEND)
		{
			if(Joystick.ReceiveFlg){
				Joystick_ClearTime_Flg = 0;
				if(Joystick.Calibration_Flg==3 || TESTMODE){
					sscanf(UART_Rx.buf,"<%d,%d,%d>",&Joystick.X,&Joystick.Y,&isMove);
					Joystick.X-= Joystick.CenterX;
					Joystick.Y-= Joystick.CenterY;     
					
					//목표 속도 계산
					calc_Goal_Speed(&motor_R.speed_Goal,&motor_L.speed_Goal,Joystick.X,Joystick.Y);
					
					//Fault Display
					HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin, HAL_GPIO_ReadPin(FAULTn_M1_GPIO_Port,FAULTn_M1_Pin));
					HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin, HAL_GPIO_ReadPin(FAULTn_M2_GPIO_Port,FAULTn_M2_Pin));
					
					if(isMove==5){
						Joystick.Calibration_Flg  =0;
						Joystick.CenterX =0;
						Joystick.CenterY =0;
					}
					
				}
				else{		//do Calibration
					joystick_cnt++;
					sscanf(UART_Rx.buf,"<%d,%d,%d>",&Joystick.X,&Joystick.Y,&isMove);
					Joystick.CenterX += Joystick.X;
					Joystick.CenterY += Joystick.Y;
					if(joystick_cnt>=JOYSTICK_CALIBRATE_TIME) {
						AngleGoal =0 ;
						if(USE_BNO055) begin_BNO055();
						Joystick.CenterX /= JOYSTICK_CALIBRATE_TIME;
						Joystick.CenterY /= JOYSTICK_CALIBRATE_TIME;
						Joystick.Calibration_Flg=3;
						joystick_cnt =0;
						
					}
				}
				Joystick.ReceiveFlg =0;
			}			
			timFlg_5ms_BTSEND=0;	
		}
		
		if(ltimFlg_1000ms)			timFlg_1000ms  = 0;
		if(ltimFlg_100ms)			timFlg_100ms    = 0;
		if(ltimFlg_10ms)				timFlg_10ms		=	0;
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 4;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_14TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3499;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 41;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 6999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 839;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 199;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin 
                          |LED1_Pin|LED0_Pin|DIR_M1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_485_Pin|nSLEEP_M1_Pin|BRK1_Pin|BRK2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CLK_Loadcell_2_Pin|CLK_Loadcell_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DIR_DC_M1_Pin|PWM_DC_M1_Pin|DIR_M2_Pin|nSLEEP_M2_Pin 
                          |BZ_CTRL_Pin|SPI_CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BT_STS_Pin SW6_Pin SW5_Pin SW4_Pin 
                           SW3_Pin */
  GPIO_InitStruct.Pin = BT_STS_Pin|SW6_Pin|SW5_Pin|SW4_Pin 
                          |SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED5_Pin LED4_Pin LED3_Pin LED2_Pin 
                           LED1_Pin LED0_Pin DIR_M1_Pin */
  GPIO_InitStruct.Pin = LED5_Pin|LED4_Pin|LED3_Pin|LED2_Pin 
                          |LED1_Pin|LED0_Pin|DIR_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B24V_V_Pin SW8_Pin SMSOUT_M2_Pin FAULTn_M2_Pin */
  GPIO_InitStruct.Pin = B24V_V_Pin|SW8_Pin|SMSOUT_M2_Pin|FAULTn_M2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_485_Pin nSLEEP_M1_Pin BRK1_Pin BRK2_Pin */
  GPIO_InitStruct.Pin = EN_485_Pin|nSLEEP_M1_Pin|BRK1_Pin|BRK2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SW7_Pin SW2_Pin SW1_Pin */
  GPIO_InitStruct.Pin = SW7_Pin|SW2_Pin|SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_Loadcell_2_Pin CLK_Loadcell_1_Pin */
  GPIO_InitStruct.Pin = CLK_Loadcell_2_Pin|CLK_Loadcell_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Data_Loadcell_2_Pin Data_Loadcell_1_Pin */
  GPIO_InitStruct.Pin = Data_Loadcell_2_Pin|Data_Loadcell_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_DC_M1_Pin PWM_DC_M1_Pin DIR_M2_Pin nSLEEP_M2_Pin 
                           BZ_CTRL_Pin SPI_CS1_Pin */
  GPIO_InitStruct.Pin = DIR_DC_M1_Pin|PWM_DC_M1_Pin|DIR_M2_Pin|nSLEEP_M2_Pin 
                          |BZ_CTRL_Pin|SPI_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SNSOUT_M1_Pin FAULTn_M1_Pin */
  GPIO_InitStruct.Pin = SNSOUT_M1_Pin|FAULTn_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
