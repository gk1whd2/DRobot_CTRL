

#if !defined(SBB_CANInit_INCLUDE_)    //헤더파일이 중복되지 않게 막아주는거야
#define SBB_CANInit_INCLUDE_

#include "main.h"
#include "Defs.h"

/**
  * @brief  CAN 설정
  * @param  hcan 		: CAN handle
  * @retval 누구세요
  */
HAL_StatusTypeDef MX_CAN_Config(CAN_HandleTypeDef* phcan);

/**
  * @brief  CAN 2.0 A 송신위한 버퍼 입력 함수
  * @param  hcan 		: CAN handle
  * @param  idTypes : CAN 2.0 A/B ID type ( CAN_ID_STD, CAN_ID_EXT)
  * @param  ID		 	: CAN 2.0 A/B ID ( 0x000 ~ 0x7FF) or  ( 0x00000000 ~ 0x1FFFFFFF )
  * @param  Buff  	: CAN 2.0 A/B 전송할 데이터 버퍼 (8 byte)
  * @retval 설정 여부		:  
											-1 :  idTypes 입력 부정확 
										 	 0 :  설정 완료
  */
HAL_StatusTypeDef CAN_WriteData_Base(CAN_HandleTypeDef* phcan, int8_t idTypes, uint32_t ID, uint8_t * Buff);	//CAN 송신


/**
  * @brief  CAN 송신
  * @param  hcan 		: CAN handle
  * @param  idTypes : CAN 2.0 A/B ID type ( CAN_ID_STD, CAN_ID_EXT)
  * @param  ID		 	: CAN 2.0 A/B ID ( 0x000 ~ 0x7FF) or  ( 0x00000000 ~ 0x1FFFFFFF )
  * @param  Buff  	: CAN 2.0 A/B 전송할 데이터 버퍼 (8 byte)
  * @retval HAL_OK
  */
HAL_StatusTypeDef CAN_WriteData(CAN_HandleTypeDef* phcan, uint8_t idTypes, uint32_t ID, uint8_t * Buff);	//CAN 송신


int Chk_CAN_VAL_Link(uint32_t canMsgID, uint8_t  *canMsgBuff);	//Joystick에서,AHRS 데이터 can으로 수신

/**
  * @brief  PSD 데이터 송신 to JOYSTICK (ID:0x91)
  * @param  pPsd : PSD 번호
  * @retval none
  */
void SBB_CAN_sendPSDdata(PSD *pPsd);				//PSD데이터전송 0x91

/**
  * @brief  목표 전압 설정 데이터 송신 to MotorDrive MoonWalker (ID:0x01)
	* @param motor : 선택할 모터 
  * @param  vol : 설정할 전압
  * @retval none
  */
void SBB_CAN_sendVoltageInst(int8_t motor, int8_t vol);

/**
  * @brief  모터 파워 온 to MotorDrive MoonWalker (ID:0x01)
  * @param  motor : 선택할 모터
  * @retval none
  */
void SBB_CAN_sendPowerOn(int8_t motor,int8_t value);				//파워 온설정 

/**
  * @brief  AHRS 데이터 송신 to JOYSTICK (ID:0x90)
  * @param  pNTAHRS_DATA : AHRS 번호
  * @retval none
  */
void SBB_CAN_sendAHRSdata(NTAHRS_DATA *pNTAHRS_DATA);		//AHRS데이터전송 0x90

/**
  * @brief  Encoder 데이터 송신 to JOYSTICK (ID:0x90)
  * @param  pLANDING_M : tilting or landing
  * @retval none
  */

void SBB_CAN_sendENCdata(LINEAR_M *pLIFT_M,LINEAR_M *pLANDING_M);		//엔코더 값,상태 송신 0x90

/**
  * @brief  can 수신 콜백 함수
  * @param  none
  * @retval none
  */
void SBB_CAN_sendSWdata(void);		//엔코더 값,상태 송신 0x92

/**
  * @brief  can 수신 콜백 함수
  * @param  hcan : can 번호
  * @retval none
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
#endif


