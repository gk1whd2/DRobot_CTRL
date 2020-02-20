#if !defined(SBB_canMW_INCLUDE_)    //헤더파일이 중복되지 않게 막아주는거야
#define SBB_canMW_INCLUDE_

#include "main.h"
#include "Defs.h"

/**
  * @brief  AHRS 데이터 검사
  * @param  canID 		: CAN ID
  * @param  canRecData  : CAN data
  * @retval 
  */
int Chk_NTAHRS_CANPacket(uint32_t canID, uint8_t *canRecData);

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
);


#endif