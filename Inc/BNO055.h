#if !defined(BNO055_INCLUDE_)    //헤더파일이 중복되지 않게 막아주는거야
#define BNO055_INCLUDE_

#include "main.h"
#include "Defs.h"


extern BNO055 bno055;


/**
  * @brief  BNO055 데이터 쓰기 - UART5
  * @param  RegAddr : 레지스터 주소
  * @param  Length   : 쓸 데이터 길이
  * @param data		  : 데이터 배열 주소
  * @retval 	쓰기 성공 여부
  */
  uint8_t reg_write(uint8_t RegAddr,uint8_t Length,uint8_t* data);
	
/**
  * @brief  BNO055 데이터 읽기 - UART5
  * @param  RegAddr : 레지스터 주소
  * @param  Length   : 읽을 데이터 길이
  * @param data		  : 저장할 데이터 배열 주소
  * @retval 	읽기 성공 여부
  */
  uint8_t reg_read(uint8_t RegAddr,uint8_t Length,uint8_t* data);
	


/**
* @brief  BNO055 초기화 - UART5
  * @retval 	
  */
void begin_BNO055();


/**
  * @brief  BNO055 벡터 요청 - UART5
  * @param  vetcor_type : 읽을 벡터 종류
  * @retval 	벡터
  */
  void requestVector(vector_type_t vector_type);

/**
  * @brief  BNO055 벡터 계산 - UART5
  * @param  vetcor_type : 읽을 벡터 종류
  * @retval 	벡터
  */
  void calcVector(vector_type_t vector_type);
  
/**
  * @brief  BNO055 Gyro 벡터 요청 - UART5
  * @param  vetcor_type : 읽을 벡터 종류
  * @retval 	벡터
  */
  void requestGyroZ();

/**
  * @brief  BNO055 Gyro 벡터 계산 - UART5
  * @param  vetcor_type : 읽을 벡터 종류
  * @retval 	벡터
  */
  void getGyroZ();

#endif