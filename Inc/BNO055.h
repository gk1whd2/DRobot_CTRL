#if !defined(BNO055_INCLUDE_)    //��������� �ߺ����� �ʰ� �����ִ°ž�
#define BNO055_INCLUDE_

#include "main.h"
#include "Defs.h"


extern BNO055 bno055;


/**
  * @brief  BNO055 ������ ���� - UART5
  * @param  RegAddr : �������� �ּ�
  * @param  Length   : �� ������ ����
  * @param data		  : ������ �迭 �ּ�
  * @retval 	���� ���� ����
  */
  uint8_t reg_write(uint8_t RegAddr,uint8_t Length,uint8_t* data);
	
/**
  * @brief  BNO055 ������ �б� - UART5
  * @param  RegAddr : �������� �ּ�
  * @param  Length   : ���� ������ ����
  * @param data		  : ������ ������ �迭 �ּ�
  * @retval 	�б� ���� ����
  */
  uint8_t reg_read(uint8_t RegAddr,uint8_t Length,uint8_t* data);
	


/**
* @brief  BNO055 �ʱ�ȭ - UART5
  * @retval 	
  */
void begin_BNO055();


/**
  * @brief  BNO055 ���� ��û - UART5
  * @param  vetcor_type : ���� ���� ����
  * @retval 	����
  */
  void requestVector(vector_type_t vector_type);

/**
  * @brief  BNO055 ���� ��� - UART5
  * @param  vetcor_type : ���� ���� ����
  * @retval 	����
  */
  void calcVector(vector_type_t vector_type);
  
/**
  * @brief  BNO055 Gyro ���� ��û - UART5
  * @param  vetcor_type : ���� ���� ����
  * @retval 	����
  */
  void requestGyroZ();

/**
  * @brief  BNO055 Gyro ���� ��� - UART5
  * @param  vetcor_type : ���� ���� ����
  * @retval 	����
  */
  void getGyroZ();

#endif