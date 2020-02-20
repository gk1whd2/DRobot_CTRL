#include "main.h"
#include "Defs.h"
//LOADCELL Loadcell;

int8_t timFlg_1us=0;
int32_t timCnt_1us=0;
int32_t timCnt_1ms = 0;

int16_t timFlg_1ms=0;
int16_t timFlg_1ms_Cur=0;
int16_t timFlg_3ms=0;
int16_t timFlg_5ms_BTSEND=0;
int16_t timFlg_10ms=0;
int16_t timFlg_10ms_Prg=0;
int16_t timFlg_100ms = 0;
int16_t timFlg_1000ms = 0;
int16_t timFlg_20ms_mid=0;	//

/**
  * @brief  타이머 시간 제어 함수
  * @param  htim 타이머 번호
  * @retval none
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int32_t usCnt_1us		= 0;
	static int32_t msCnt_1ms		= 0;
	static int32_t msCnt_3ms		= 0;
	static int32_t msCnt_5ms		= 0;
	static int32_t msCnt_10ms		= 0;
	static int32_t msCnt_20ms_mid = 0;
	static int32_t msCnt_100ms	= 0;
	static int32_t msCnt_1000ms	= 0;
	
	if(htim->Instance==TIM10)
	{
		msCnt_10ms++;
		msCnt_100ms++;
		msCnt_1000ms++;
		msCnt_3ms++;
		msCnt_5ms++;
		msCnt_1ms++;
		timCnt_1ms++;
		msCnt_20ms_mid++;
			
		if(msCnt_1ms>=1)
		{
			timFlg_1ms=1;
			timFlg_1ms_Cur=1;
			msCnt_1ms=0;
		}
		
		if(msCnt_3ms >=3){
			timFlg_3ms=1;
			msCnt_3ms =0;
		}

		if(msCnt_5ms>=5)
		{
			msCnt_5ms=0;
		}
		if(msCnt_10ms >= 10)
		{
			timFlg_5ms_BTSEND = 1;
			timFlg_10ms=1;
			timFlg_10ms_Prg = 1;
			msCnt_10ms=0;
		}
		
		if(msCnt_20ms_mid >= 20){
			timFlg_20ms_mid = 1;
			msCnt_20ms_mid = 0;
		}
		if(msCnt_100ms	>= 100)
		{
			timFlg_100ms=1;
			msCnt_100ms=0;
		}
		if(msCnt_1000ms	>= 1000)
		{
			timFlg_1000ms=1;
			msCnt_1000ms=0;
		}
	}
	else if(htim->Instance==TIM7)
	{
		usCnt_1us++;
		timCnt_1us++;
		timFlg_1us = 1;
	}
}


