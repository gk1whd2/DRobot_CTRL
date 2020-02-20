#include "main.h"
#include "Defs.h"
#include "SBB_canNTAHRS.h"

extern CAN_HandleTypeDef hcan2;

extern NTAHRS_DATA		NTAHRSData_Body;		//ID:10
extern NTAHRS_DATA		NTAHRSData_Seat;		//ID:11

HAL_StatusTypeDef CAN_WriteData_Base(CAN_HandleTypeDef* phcan, uint8_t idTypes, uint32_t ID, uint8_t * Buff);
HAL_StatusTypeDef CAN_WriteData(CAN_HandleTypeDef* phcan, uint8_t idTypes, uint32_t ID, uint8_t * Buff);	//CAN 송신

/**
  * @brief  AHRS 데이터 검사
  * @param  canID 		: CAN ID
  * @param  canRecData  : CAN data
  * @retval 
  */
int Chk_NTAHRS_CANPacket(uint32_t canID, uint8_t *canRecData)			//AHRS데잍ㅓ검사하는 애
{
	if(!((canID == CAN_ID_DEV_AHRS_ID_1)||(canID == CAN_ID_DEV_AHRS_ID_2)))
		return -1;	
	
	switch(canRecData[0])
	{
		case 0xF0:
			break;
		default:
			return -2;
	}
	
	switch(canRecData[1])
	{
		case 0x33:
		case 0x34:
		case 0x35:
		case 0x36:
			break;
		default:
			return -3;
	}
	return 1;	
}

/**
  * @brief  CAN 데이터 수신_AHRS 데이터 분류
  * @param  pNTAHRS_DATA : AHRS 분류
  * @param  canID		 : CAN ID
  * @param  pNTAHRS_DATA : 수신된 AHRS 데이터
  * @retval none
  */

int Parssing_NTAHRS_CANPacket(												//AHRS데잍ㅓ변환
		NTAHRS_DATA *pNTAHRS_DATA
	, uint32_t canID
	, uint8_t *canRecData
)
{
	if(Chk_NTAHRS_CANPacket(canID, canRecData)	<0)
		return -1;
	
	
	switch(canRecData[1])
	{
		case 0x33:	// Acc
			pNTAHRS_DATA->raw_acc[axis_X] = canRecData[3];
			pNTAHRS_DATA->raw_acc[axis_X] = pNTAHRS_DATA->raw_acc[axis_X]<<8;
			pNTAHRS_DATA->raw_acc[axis_X] |= canRecData[2];
		
			pNTAHRS_DATA->raw_acc[axis_Y] = canRecData[5];
			pNTAHRS_DATA->raw_acc[axis_Y] = pNTAHRS_DATA->raw_acc[axis_Y]<<8;
			pNTAHRS_DATA->raw_acc[axis_Y] |= canRecData[4];
		
			pNTAHRS_DATA->raw_acc[axis_Z] = canRecData[7];
			pNTAHRS_DATA->raw_acc[axis_Z] = pNTAHRS_DATA->raw_acc[axis_Z]<<8;
			pNTAHRS_DATA->raw_acc[axis_Z] |= canRecData[6];
		
			pNTAHRS_DATA->conv_acc[axis_X] = pNTAHRS_DATA->raw_acc[axis_X]/1000;
			pNTAHRS_DATA->conv_acc[axis_Y] = pNTAHRS_DATA->raw_acc[axis_Y]/1000;
			pNTAHRS_DATA->conv_acc[axis_Z] = pNTAHRS_DATA->raw_acc[axis_Z]/1000;		
		
			break;
		
		case 0x34:	// Gyro
			pNTAHRS_DATA->raw_gyro[axis_X] = canRecData[3];
			pNTAHRS_DATA->raw_gyro[axis_X] = pNTAHRS_DATA->raw_gyro[axis_X]<<8;
			pNTAHRS_DATA->raw_gyro[axis_X] |= canRecData[2];
		
			pNTAHRS_DATA->raw_gyro[axis_Y] = canRecData[5];
			pNTAHRS_DATA->raw_gyro[axis_Y] = pNTAHRS_DATA->raw_gyro[axis_Y]<<8;
			pNTAHRS_DATA->raw_gyro[axis_Y] |= canRecData[4];
		
			pNTAHRS_DATA->raw_gyro[axis_Z] = canRecData[7];
			pNTAHRS_DATA->raw_gyro[axis_Z] = pNTAHRS_DATA->raw_gyro[axis_Z]<<8;
			pNTAHRS_DATA->raw_gyro[axis_Z] |= canRecData[6];
		
			pNTAHRS_DATA->conv_gyro[axis_X] = pNTAHRS_DATA->raw_gyro[axis_X]/10;
			pNTAHRS_DATA->conv_gyro[axis_Y] = pNTAHRS_DATA->raw_gyro[axis_Y]/10;
			pNTAHRS_DATA->conv_gyro[axis_Z] = pNTAHRS_DATA->raw_gyro[axis_Z]/10;		
			break;
		
		case 0x35:	// Euler
			pNTAHRS_DATA->raw_euler[eulerAxis_PITCH] = canRecData[3];
			pNTAHRS_DATA->raw_euler[eulerAxis_PITCH] = pNTAHRS_DATA->raw_euler[eulerAxis_PITCH]<<8;
			pNTAHRS_DATA->raw_euler[eulerAxis_PITCH] |= canRecData[2];
		
			pNTAHRS_DATA->raw_euler[eulerAxis_ROLL] = canRecData[5];
			pNTAHRS_DATA->raw_euler[eulerAxis_ROLL] = pNTAHRS_DATA->raw_euler[eulerAxis_ROLL]<<8;
			pNTAHRS_DATA->raw_euler[eulerAxis_ROLL] |= canRecData[4];
		
			pNTAHRS_DATA->raw_euler[eulerAxis_YAW] = canRecData[7];
			pNTAHRS_DATA->raw_euler[eulerAxis_YAW] = pNTAHRS_DATA->raw_euler[eulerAxis_YAW]<<8;
			pNTAHRS_DATA->raw_euler[eulerAxis_YAW] |= canRecData[6];
		
		
			pNTAHRS_DATA->conv_euler[eulerAxis_PITCH] = ((float)pNTAHRS_DATA->raw_euler[eulerAxis_PITCH]/100 - pNTAHRS_DATA->raw_euler_Offset[eulerAxis_PITCH]);
			pNTAHRS_DATA->conv_euler[eulerAxis_ROLL] 	= ((float)pNTAHRS_DATA->raw_euler[eulerAxis_ROLL]/100 - pNTAHRS_DATA->raw_euler_Offset[eulerAxis_ROLL]);
			pNTAHRS_DATA->conv_euler[eulerAxis_YAW] 	= ((float)pNTAHRS_DATA->raw_euler[eulerAxis_YAW]/100 - pNTAHRS_DATA->raw_euler_Offset[eulerAxis_YAW]);
			break;
		
		case 0x36:	// Mag
			pNTAHRS_DATA->raw_mag[axis_X] = canRecData[3];
			pNTAHRS_DATA->raw_mag[axis_X] = pNTAHRS_DATA->raw_mag[axis_X]<<8;
			pNTAHRS_DATA->raw_mag[axis_X] |= canRecData[2];
		
			pNTAHRS_DATA->raw_mag[axis_Y] = canRecData[5];
			pNTAHRS_DATA->raw_mag[axis_Y] = pNTAHRS_DATA->raw_mag[axis_Y]<<8;
			pNTAHRS_DATA->raw_mag[axis_Y] |= canRecData[4];
		
			pNTAHRS_DATA->raw_mag[axis_Z] = canRecData[7];
			pNTAHRS_DATA->raw_mag[axis_Z] = pNTAHRS_DATA->raw_mag[axis_Z]<<8;
			pNTAHRS_DATA->raw_mag[axis_Z] |= canRecData[6];
			break;
	}

	pNTAHRS_DATA->updateFlg = 1;
	pNTAHRS_DATA->updatedTime = 0;
	return 0;	
}
