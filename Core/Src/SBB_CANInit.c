#include "main.h"
#include "Defs.h"
#include "SBB_CANInit.h"


extern CAN_HandleTypeDef hcan1;

extern int Parssing_NTAHRS_CANPacket(NTAHRS_DATA *pNTAHRS_DATA, uint32_t canID, uint8_t *canRecData);

/**
  * @brief  CAN 설정
  * @param  hcan 		: CAN handle
  * @retval 누구세요
  */
HAL_StatusTypeDef MX_CAN_Config(CAN_HandleTypeDef* phcan)
{
  CAN_FilterTypeDef  sFilterConfig;
	HAL_StatusTypeDef res = HAL_ERROR;

	//can2 사용하려면 이렇게 해야함
	if(phcan->Instance == CAN1){
		sFilterConfig.FilterBank = 0;
	}else if(phcan->Instance == CAN2){
		sFilterConfig.FilterBank = 14;
	}else{
		return HAL_ERROR;
	}
  
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0;           // Ici, 320 est l'adresse de la carte. Il peux etre different pour chaque carte.
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0;       // Le masque peux servir a accepter une plage d'adresse au lieu d'une adresse unique.
  sFilterConfig.FilterMaskIdLow = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(phcan, &sFilterConfig);      // Configure le filtre comme ci-dessus
	
		
  res = HAL_CAN_Start (phcan); 
	if(res != HAL_OK)
		return res;

  res = HAL_CAN_ActivateNotification (phcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
	if(res != HAL_OK)
		return res;
	
	return HAL_OK;	
}


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
HAL_StatusTypeDef CAN_WriteData_Base(CAN_HandleTypeDef* phcan, int8_t idTypes, uint32_t ID, uint8_t * Buff)	//CAN 송신
{

	HAL_StatusTypeDef canRes;
//	char str[100] = {0,};

	static CAN_TxHeaderTypeDef   TxHeader;
	static uint8_t               TxData[8] = {0,};
	static uint32_t              TxMailbox;

  TxHeader.StdId = 0x000; 
  TxHeader.ExtId = 0x00; 
  TxHeader.RTR = CAN_RTR_DATA; 
  TxHeader.IDE = CAN_ID_STD; 
  TxHeader.DLC = 8; 
  TxHeader.TransmitGlobalTime = DISABLE;
	
	
	switch(idTypes){
		case CAN_ID_STD:	TxHeader.IDE = idTypes; TxHeader.StdId = ID; 			break;
		case CAN_ID_EXT:	TxHeader.IDE = idTypes; TxHeader.ExtId = ID; 			break;	
		default:
			return -1;
	}
	for(int i = 0; i < 8; i++)
  {
		TxData[i] = Buff[i];
  }
	
  canRes = HAL_CAN_AddTxMessage(phcan, &TxHeader, TxData, &TxMailbox);
	return canRes;	
}

/**
  * @brief  CAN 송신
  * @param  hcan 		: CAN handle
  * @param  idTypes : CAN 2.0 A/B ID type ( CAN_ID_STD, CAN_ID_EXT)
  * @param  ID		 	: CAN 2.0 A/B ID ( 0x000 ~ 0x7FF) or  ( 0x00000000 ~ 0x1FFFFFFF )
  * @param  Buff  	: CAN 2.0 A/B 전송할 데이터 버퍼 (8 byte)
  * @retval HAL_OK
  */
HAL_StatusTypeDef CAN_WriteData(CAN_HandleTypeDef* phcan, uint8_t idTypes, uint32_t ID, uint8_t * Buff)	//CAN 송신
{
	HAL_StatusTypeDef res;
	
	do{
		res = CAN_WriteData_Base(phcan, idTypes, ID, Buff);
	}while(res != HAL_OK);
	return res;
}

int Chk_CAN_VAL_Link(uint32_t canMsgID, uint8_t  *canMsgBuff)	//Joystick에서,AHRS 데이터 can으로 수신
{
	uint16_t temp_u16;	
	int16_t  temp_16;
		switch(canMsgID){
			case CAN_ID_DEV_AHRS_ID_1:
				//Parssing_NTAHRS_CANPacket(&NTAHRSData_Body,canMsgID,canMsgBuff);
				break;
			
			case CAN_ID_DEV_AHRS_ID_2:
				//Parssing_NTAHRS_CANPacket(&NTAHRSData_Seat,canMsgID,canMsgBuff);			
				break;
			
			
			case 0x081:
				temp_u16 = canMsgBuff[3];
				temp_u16 = (temp_u16<<8) | canMsgBuff[2];
				//RecJoyData.sts_rec_Vl = temp_u16;
				
				temp_u16 = canMsgBuff[5];
				temp_u16 = (temp_u16<<8) | canMsgBuff[4];
				//RecJoyData.sts_rec_Vr = temp_u16;
				
				//RecJoyData.updateFlg_Cat = 1;
				break;
			
			case 0x082:
				temp_u16 = canMsgBuff[3];
				temp_u16 = (temp_u16<<8) | canMsgBuff[2];
				//RecJoyData.sts_rec_Lift_Tilt = temp_u16;
//
				temp_u16 = canMsgBuff[5];
				//temp_u16 = (temp_u16<<8) | canMsgBuff[4];
				//RecJoyData.sts_rec_Land_Tilt = temp_u16;
			
				//RecJoyData.updateFlg_Linear = 1;
				break;
			
			case 0x080:
				temp_u16 = canMsgBuff[1];
				temp_u16 = (temp_u16<<8) | canMsgBuff[0];
				//RecJoyData.sts_rec_Heartbeat = temp_u16;
				
			
	// //			RecJoyData.updateFlg = 1;
				break;
			
			case 0x001:
				if(canMsgBuff[0] == MW_CMD_READ_RES_I32_x48){
					temp_u16 = canMsgBuff[2];
					temp_u16 = (temp_u16<<8) | canMsgBuff[1];
					
					if(temp_u16 == MWIDEX_STS_POSITION_125)
					{
						if(canMsgBuff[3] == LIFT_BODY)
						{
					
							temp_u16 = canMsgBuff[5];
							temp_u16 = (temp_u16<<8) | canMsgBuff[4];
							//Landing_M.sts_Pos = temp_u16;
						}
					}
				}				
				break;// 신기한 지안이의 코딩 
				
			case 0x085:
				if(canMsgBuff[0]==0x18){
					// //write int32_t

				}
				else if(canMsgBuff[0]==0x38){
					int sIdx =  canMsgBuff[3];					
					
					// //read int 32_t
					//Tilting[sIdx].stsCMD=stsRead;
					
					temp_u16 = canMsgBuff[2];
					temp_u16 = (temp_u16<<8) | canMsgBuff[1];
					//Tilting[sIdx].cmdIndex=temp_u16;
					
					temp_16 = canMsgBuff[5];
					temp_16 = (temp_16<<8) | canMsgBuff[4];
					//Tilting[sIdx].Value = temp_16;
				}
				break;
	
			case 0x090:
				if(canMsgBuff[0] == 1){
					temp_u16 = canMsgBuff[3];
					temp_u16 = (temp_u16<<8) | canMsgBuff[2];
					//NTAHRSData_Body.conv_euler[eulerAxis_ROLL]=(temp_u16)/100+((temp_u16)%100)*0.01;
					
					temp_u16 = canMsgBuff[5];
					temp_u16 = (temp_u16<<8) | canMsgBuff[4];
					//NTAHRSData_Body.conv_euler[eulerAxis_PITCH]=(temp_u16)/100+((temp_u16)%100)*0.01;
					
					temp_u16 = canMsgBuff[5];
					temp_u16 = (temp_u16<<8) | canMsgBuff[4];
					//NTAHRSData_Body.conv_euler[eulerAxis_YAW]=(temp_u16);
				}
				else if(canMsgBuff[0] == 2){
					temp_u16 = canMsgBuff[3];
					temp_u16 = (temp_u16<<8) | canMsgBuff[2];
					//NTAHRSData_Seat.conv_euler[eulerAxis_ROLL]=temp_u16;
					
					temp_u16 = canMsgBuff[5];
					temp_u16 = (temp_u16<<8) | canMsgBuff[4];
					//NTAHRSData_Seat.conv_euler[eulerAxis_PITCH]=temp_u16;
					
					temp_u16 = canMsgBuff[5];
					temp_u16 = (temp_u16<<8) | canMsgBuff[4];
					//NTAHRSData_Seat.conv_euler[eulerAxis_YAW]=temp_u16;
				}
				break;
		}

	
	return 0;
}


/**
  * @brief  can 수신 콜백 함수
  * @param  hcan : can 번호
  * @retval none
  */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t recID = 0;
	static CAN_RxHeaderTypeDef   RxHeader;
	static uint8_t               RxData[8];

	
  HAL_CAN_GetRxMessage (hcan, CAN_RX_FIFO0, & RxHeader, RxData);
	
	if(					RxHeader.IDE == CAN_ID_STD){
		recID = RxHeader.StdId;  
	}else if(		RxHeader.IDE == CAN_ID_EXT){
		recID = RxHeader.ExtId;  
	}
		Chk_CAN_VAL_Link(recID, RxData);
}



/**
  * @brief  PSD 데이터 송신 to JOYSTICK (ID:0x91)
  * @param  pPsd : PSD 번호
  * @retval none
  */

void SBB_CAN_sendPSDdata(PSD *pPsd)				//PSD데이터전송 0x91
{
	const int commandID = CAN_ID_M2_SURF_DETECT_x91;
	static uint8_t canTxBuff[8] = {0,};
	
	canTxBuff[0] = 1;
	canTxBuff[1] = 0;
	
	canTxBuff[2] = (pPsd[0].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[3] = (pPsd[0].sts_Distance>>8)&0xff;
	
	canTxBuff[4] = (pPsd[1].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[5] = (pPsd[1].sts_Distance>>8)&0xff;

	canTxBuff[6] = (pPsd[2].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[7] = (pPsd[2].sts_Distance>>8)&0xff;
	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);

	canTxBuff[0] = 2;
	canTxBuff[1] = 0;
	
	canTxBuff[2] = (pPsd[3].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[3] = (pPsd[3].sts_Distance>>8)&0xff;
	
	canTxBuff[4] = (pPsd[4].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[5] = (pPsd[4].sts_Distance>>8)&0xff;

	canTxBuff[6] = (pPsd[5].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[7] = (pPsd[5].sts_Distance>>8)&0xff;
	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);
/*
	canTxBuff[0] = 3;
	canTxBuff[1] = 0;
	
	canTxBuff[2] = (pPsd[6].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[3] = (pPsd[6].sts_Distance>>8)&0xff;
	
	canTxBuff[4] = (pPsd[7].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[5] = (pPsd[7].sts_Distance>>8)&0xff;

	canTxBuff[6] = (pPsd[8].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[7] = (pPsd[8].sts_Distance>>8)&0xff;
	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);

	canTxBuff[0] = 4;
	canTxBuff[1] = 0;
	
	canTxBuff[2] = (pPsd[9].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[3] = (pPsd[9].sts_Distance>>8)&0xff;
	
	canTxBuff[4] = (pPsd[10].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[5] = (pPsd[10].sts_Distance>>8)&0xff;

	canTxBuff[6] = (pPsd[11].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[7] = (pPsd[11].sts_Distance>>8)&0xff;
	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);
*/
}

/**
  * @brief  목표 전압 설정 데이터 송신 to MotorDrive MoonWalker (ID:0x01)
  * @param  vol : 설정할 전압
  * @retval none
  */
void SBB_CAN_sendVoltageInst(int8_t motor, int8_t vol)				//목표 전압 설정 데이터전송 0x01
{
	const int commandID = motor;
	static int8_t canTxBuff[8] = {0,};
	
	canTxBuff[0] = 0x10;
	canTxBuff[1] = 114;
	canTxBuff[2] = 0;
	canTxBuff[3] = 1;
	canTxBuff[4] = vol;
	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);

/*
	canTxBuff[0] = 3;
	canTxBuff[1] = 0;
	
	canTxBuff[2] = (pPsd[6].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[3] = (pPsd[6].sts_Distance>>8)&0xff;
	
	canTxBuff[4] = (pPsd[7].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[5] = (pPsd[7].sts_Distance>>8)&0xff;

	canTxBuff[6] = (pPsd[8].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[7] = (pPsd[8].sts_Distance>>8)&0xff;
	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);

	canTxBuff[0] = 4;
	canTxBuff[1] = 0;
	
	canTxBuff[2] = (pPsd[9].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[3] = (pPsd[9].sts_Distance>>8)&0xff;
	
	canTxBuff[4] = (pPsd[10].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[5] = (pPsd[10].sts_Distance>>8)&0xff;

	canTxBuff[6] = (pPsd[11].sts_Distance>>0)&0xff;  //지금은 rawData로 해놓고
	canTxBuff[7] = (pPsd[11].sts_Distance>>8)&0xff;
	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);
*/
}

/**
  * @brief  모터 파워 온 to MotorDrive MoonWalker (ID:0x01)
  * @param  none
  * @retval none
  */
void SBB_CAN_sendPowerOn(int8_t motor,int8_t value){				//파워 온설정 
	const int commandID = motor;
	static uint8_t canTxBuff[8] = {0,};
	
	canTxBuff[0] = 0x10;
	canTxBuff[1] = 101;
	canTxBuff[2] = 0;
	canTxBuff[3] = 1;
	canTxBuff[4] = value;
	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);
}

/**
  * @brief  AHRS 데이터 송신 to JOYSTICK (ID:0x90)
  * @param  pNTAHRS_DATA : AHRS 번호
  * @retval none
  */

void SBB_CAN_sendAHRSdata(NTAHRS_DATA *pNTAHRS_DATA)		//AHRS데이터전송 0x90
{
	const int commandID = CAN_ID_M2_POSITION_x90;
	uint8_t canTxBuff[8] = {0,};
	
//	if(pNTAHRS_DATA == &NTAHRSData_Body)
//	{
//			canTxBuff[0] = 1;
//			canTxBuff[1] = 0;
//		
//			canTxBuff[2] = ((int16_t)NTAHRSData_Seat.conv_euler[eulerAxis_ROLL]>>0)&0xff;  
//			canTxBuff[3] = ((int16_t)NTAHRSData_Seat.conv_euler[eulerAxis_ROLL]>>8)&0xff;
//			
//			canTxBuff[4] = ((int16_t)NTAHRSData_Seat.conv_euler[eulerAxis_PITCH]>>0)&0xff;  
//			canTxBuff[5] = ((int16_t)NTAHRSData_Seat.conv_euler[eulerAxis_PITCH]>>8)&0xff;

//			canTxBuff[6] = ((int16_t)NTAHRSData_Seat.conv_euler[2]>>0)&0xff;  
//			canTxBuff[7] = ((int16_t)NTAHRSData_Seat.conv_euler[2]>>8)&0xff;			

//			
//			CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);
//	}
//	else if(pNTAHRS_DATA == &NTAHRSData_Seat)
//	{
//			canTxBuff[0] = 2;
//			canTxBuff[1] = 0;
//		
//			canTxBuff[2] = ((int16_t)NTAHRSData_Body.conv_euler[eulerAxis_ROLL]>>0)&0xff;  
//			canTxBuff[3] = ((int16_t)NTAHRSData_Body.conv_euler[eulerAxis_ROLL]>>8)&0xff;
//			
//			canTxBuff[4] = ((int16_t)NTAHRSData_Body.conv_euler[eulerAxis_PITCH]>>0)&0xff;  
//			canTxBuff[5] = ((int16_t)NTAHRSData_Body.conv_euler[eulerAxis_PITCH]>>8)&0xff;

//			canTxBuff[6] = ((int16_t)NTAHRSData_Body.conv_euler[2]>>0)&0xff;  
//			canTxBuff[7] = ((int16_t)NTAHRSData_Body.conv_euler[2]>>8)&0xff;			

//			
//			CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);
//	}
	
}

/**
  * @brief  Encoder 데이터 송신 to JOYSTICK (ID:0x90)
  * @param  pLANDING_M : tilting or landing
  * @retval none
  */

void SBB_CAN_sendENCdata(LINEAR_M *pLIFT_M,LINEAR_M *pLANDING_M)		//엔코더 값,상태 송신 0x90
{
	const int commandID = CAN_ID_M2_POSITION_x90;
//	const int commandID = 0x82;
	uint8_t canTxBuff[8] = {0,};

	
	canTxBuff[0] = 3;
	canTxBuff[1] = 0;	
//	if(pLIFT_M == &Lift_M){
//		canTxBuff[2] = ((int16_t)Lift_M.sts_Pos>>0)&0xff;  
//		canTxBuff[3] = ((int16_t)Lift_M.sts_Pos>>8)&0xff;
//		canTxBuff[6] = (int8_t)Lift_M.status_ENC;  
//	}
//	if(pLANDING_M == &Landing_M){	
//		canTxBuff[4] = ((int16_t)Landing_M.sts_Pos>>0)&0xff;  
//		canTxBuff[5] = ((int16_t)Landing_M.sts_Pos>>8)&0xff;

//		canTxBuff[7] = (int8_t)Landing_M.status_ENC;		
//	}
//	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);
}

void SBB_CAN_sendSWdata(void)		//엔코더 값,상태 송신 0x92
{
	const int commandID = CAN_ID_M2_LIMITSWITCH_x92;
	uint8_t canTxBuff[8] = {0,};

	
	canTxBuff[0] = 1;
	canTxBuff[1] = 0;	
	
	//canTxBuff[2] = Tilting[S_INDEX_BODY].Limit_Body_H;  
	canTxBuff[3] = 0;

	//canTxBuff[4] = Tilting[S_INDEX_BODY].Limit_Body_L;  
	canTxBuff[5] = 0;

	//canTxBuff[6] = Tilting[S_INDEX_SEAT].Limit_Seat_H;  
	canTxBuff[7] = 0;

	
	CAN_WriteData(&hcan1,CAN_ID_STD,commandID,canTxBuff);
}