#if !defined(SBB_rccTIM_INCLUDE_)    //헤더파일이 중복되지 않게 막아주는거야
#define SBB_rccTIM_INCLUDE_

#include "main.h"
#include "Defs.h"

extern int16_t timFlg_1us;
extern int32_t timCnt_1us;
extern int32_t timCnt_1ms;

extern int16_t timFlg_1ms;
extern int16_t timFlg_1ms_Cur;
extern int16_t timFlg_3ms;
extern int16_t timFlg_5ms_BTSEND;
extern int16_t timFlg_10ms;
extern int16_t timFlg_10ms_Prg;
extern int16_t timFlg_100ms;
extern int16_t timFlg_1000ms;
extern int16_t timFlg_20ms_mid;	

/**
  * @brief  타이머 시간 제어 함수
  * @param  htim 타이머 번호
  * @retval none
  */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif