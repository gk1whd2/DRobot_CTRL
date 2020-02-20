#include "main.h"
#include "Defs.h"

BNO055 bno055;

/**
  * @brief  BNO055 데이터 쓰기 - UART5
  * @param  RegAddr : 레지스터 주소
  * @param  Length   : 쓸 데이터 길이
  * @param data		  : 데이터 배열 주소
  * @retval 	쓰기 성공 여부
  */
  uint8_t reg_write(uint8_t RegAddr,uint8_t Length,uint8_t* data){
	  uint8_t buf[20];
	  uint8_t cnt=0;
	  
	  buf[0] = 0xAA;		//START Byte
	  buf[1] = 0x00;		//Write Byte
	  buf[2] = RegAddr;
	  buf[3] = Length;
	
	  for(cnt=0;cnt<Length;cnt++)
		buf[4+cnt] = data[cnt];
	  
	 bno055.State = BNO_Write;
	  bno055.Uart5.RxFlg =0;
	  bno055.Uart5.idx =0;
	  Send_UART5_data(buf,Length+4);		
  }
  /**
  * @brief  BNO055 데이터 쓰기 - UART5
  * @param  RegAddr : 레지스터 주소
  * @param  Length   : 쓸 데이터 길이
  * @param data		  : 데이터 배열 주소
  * @retval 	쓰기 성공 여부
  */
  uint8_t reg_write8(uint8_t RegAddr,uint8_t data){
	  uint8_t buf[20];
	  uint8_t cnt=0;
	  
	  buf[0] = 0xAA;		//START Byte
	  buf[1] = 0x00;		//Write Byte
	  buf[2] = RegAddr;
	  buf[3] = 1;
	  buf[4] = data;
	  
	 bno055.State = BNO_Write;
	  bno055.Uart5.RxFlg =0;
	  bno055.Uart5.idx =0;
	  Send_UART5_data(buf,5);		
  }
	
/**
  * @brief  BNO055 데이터 읽기 - UART5
  * @param  RegAddr : 레지스터 주소
  * @param  Length   : 읽을 데이터 길이
  * @param data		  : 저장할 데이터 배열 주소
  * @retval 	읽기 성공 여부
  */
  uint8_t reg_read(uint8_t RegAddr,uint8_t Length){
	  uint8_t buf[5];
	  
	  buf[0] = 0xAA;
	  buf[1] = 0x01;
	  buf[2] = RegAddr;
	  buf[3] = Length;
	   
	  bno055.State = BNO_Read;
	  bno055.Uart5.RxFlg =0;
	  bno055.Uart5.idx =0;
	  Send_UART5_data(buf,4);
  }

/**
* @brief  BNO055 초기화 - UART5
  * @retval 	
  */
void begin_BNO055(){
	
	reg_read(BNO055_CHIP_ID_ADDR,1);
	while(!(bno055.Uart5.RxFlg));
	if(bno055.Uart5.buf[2]!=BNO055_ID) return ;
	
	//Set Mode
	reg_write8(BNO055_OPR_MODE_ADDR,OPERATION_MODE_CONFIG);
	while(!(bno055.Uart5.byte.flg)) ;
	HAL_Delay(30);
	
	
	//Reset
	reg_write8(BNO055_SYS_TRIGGER_ADDR,0x20);
	while(!(bno055.Uart5.byte.flg));
	 //Send_UART4_data("0",2);
	HAL_Delay(650);
	 //Send_UART4_data("1",2);
	
	reg_read(BNO055_CHIP_ID_ADDR,1);
	while(!(bno055.Uart5.RxFlg));
	 //Send_UART4_data("2",2);
	
	while(bno055.Uart5.buf[2]!=BNO055_ID){
		HAL_Delay(10);
	reg_read(BNO055_CHIP_ID_ADDR,1);
		while(!(bno055.Uart5.RxFlg));
	}
	HAL_Delay(50);
		
	//set Normal Power Mode
	reg_write8(BNO055_PWR_MODE_ADDR,0);//POWER_MODE_NORMAL = 0
	while(!(bno055.Uart5.byte.flg));
	HAL_Delay(10);
	
	reg_write8(BNO055_PAGE_ID_ADDR,0);
	while(!(bno055.Uart5.byte.flg));
	
	reg_write8(BNO055_SYS_TRIGGER_ADDR,0);
	while(!(bno055.Uart5.byte.flg));
	HAL_Delay(10);
	
	reg_write8(BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF);
	while(!(bno055.Uart5.RxFlg));
	HAL_Delay(20);
	
	reg_write8(BNO055_OPR_MODE_ADDR,OPERATION_MODE_CONFIG);
	while(!(bno055.Uart5.RxFlg));
	HAL_Delay(25);
	
	reg_write8(BNO055_PAGE_ID_ADDR,0);
	while(!(bno055.Uart5.RxFlg));
	
	//use External Xtal 
	reg_write8(BNO055_SYS_TRIGGER_ADDR,0x80);
	while(!(bno055.Uart5.RxFlg));
	HAL_Delay(10);
	
	reg_write8(BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF);
	while(!(bno055.Uart5.RxFlg));
	HAL_Delay(20);
	
	
	bno055.VectorState = BNO_Vector_Comp;
}



/**
  * @brief  BNO055 벡터 읽기 - UART5
  * @param  vetcor_type : 읽을 벡터 종류
  * @retval 	벡터
  */
  void getVector(vector_type_t vector_type){
	   int16_t x,y,z;
	  
	  reg_read(vector_type,6);
	  while(!(bno055.Uart5.RxFlg));
	  
	  if(bno055.State == BNO_Error){
		  //bno055.vec_Data.vector.x =-1;
		  //bno055.vec_Data.vector.y =-1;
		  //bno055.vec_Data.vector.z =-1;
		  return ;
	  }
	  
	  x = ((int16_t)bno055.Uart5.buf[2]) | (((int16_t)bno055.Uart5.buf[3])<<8);
	  y = ((int16_t)bno055.Uart5.buf[4]) | (((int16_t)bno055.Uart5.buf[5])<<8);
	  z = ((int16_t)bno055.Uart5.buf[6]) | (((int16_t)bno055.Uart5.buf[7])<<8);
	  
	  switch(vector_type)
	  {
		  case VECTOR_ACCELEROMETER:
			  bno055.vec_Data.vector.x = ((double)x) / 100.0;
			  bno055.vec_Data.vector.y = ((double)y) / 100.0;
			  bno055.vec_Data.vector.z = ((double)z) / 100.0;
			  bno055.vec_Data.updated =1;
			  break;
		  case VECTOR_MAGNETOMETER:
			  bno055.vec_Data.vector.x = ((double)x) / 16.0;
			  bno055.vec_Data.vector.y = ((double)y) / 16.0;
			  bno055.vec_Data.vector.z = ((double)z) / 16.0;
			  bno055.vec_Data.updated =1;
		  case VECTOR_GYROSCOPE:
			  bno055.vec_Data.vector.x = ((double)x) / 16.0;
			  bno055.vec_Data.vector.y = ((double)y) / 16.0;
			  bno055.vec_Data.vector.z = ((double)z) / 16.0;
			  bno055.vec_Data.updated =1;
			  break;
		  case VECTOR_EULER:
			  bno055.vec_Data.vector.x = ((double)x) / 16.0;
			  bno055.vec_Data.vector.y = ((double)y) / 16.0;
			  bno055.vec_Data.vector.z = ((double)z) / 16.0;
			  bno055.vec_Data.updated =1;
			  break;
		  case VECTOR_LINEARACCEL:
			  bno055.vec_Data.vector.x = ((double)x) / 100.0;
			  bno055.vec_Data.vector.y = ((double)y) / 100.0;
			  bno055.vec_Data.vector.z = ((double)z) / 100.0;
			  bno055.vec_Data.updated =1;
			  break;
		  case VECTOR_GRAVITY:
			  bno055.vec_Data.vector.x = ((double)x) / 100.0;
			  bno055.vec_Data.vector.y = ((double)y) / 100.0;
			  bno055.vec_Data.vector.z = ((double)z) / 100.0;
			  bno055.vec_Data.updated =1;
		  default : 
			  bno055.vec_Data.updated =0;
			  break;
	  }
  }
  /**
  * @brief  BNO055 벡터 요청 - UART5
  * @param  vetcor_type : 읽을 벡터 종류
  * @retval 	벡터
  */
  void requestVector(vector_type_t vector_type){	  
	  bno055.VectorState = BNO_Vector;
	  reg_read(vector_type,6);
  }
  void requestGyroZ(){
	  bno055.VectorState = BNO_Vector;
	  reg_read(BNO055_GYRO_DATA_Z_LSB_ADDR,2);
  }
  void getGyroZ(){
	  int16_t gyroZ;
	  if(bno055.State == BNO_Error) return;
	  
	  gyroZ = ((int16_t)bno055.Uart5.buf[2]) | (((int16_t)bno055.Uart5.buf[3])<<8);
	  
	  bno055.vec_Data.vector.x = 0;
	  bno055.vec_Data.vector.y = 0;
	  bno055.vec_Data.vector.z = ((double)gyroZ) / 16.0;
	  
	  bno055.vec_Data.updated =1;
  }
  /**
  * @brief  BNO055 벡터 계산 - UART5
  * @param  vetcor_type : 읽을 벡터 종류
  * @retval 	벡터
  */
  void calcVector(vector_type_t vector_type){
	   int16_t x,y,z;
	  
	  if(bno055.State == BNO_Error){
		 // bno055.vec_Data.vector.x =-1;
		  //bno055.vec_Data.vector.y =-1;
		  //bno055.vec_Data.vector.z =bno055.Uart5.buf[1];
		  return ;
	  }
	  
	  x = ((int16_t)bno055.Uart5.buf[2]) | (((int16_t)bno055.Uart5.buf[3])<<8);
	  y = ((int16_t)bno055.Uart5.buf[4]) | (((int16_t)bno055.Uart5.buf[5])<<8);
	  z = ((int16_t)bno055.Uart5.buf[6]) | (((int16_t)bno055.Uart5.buf[7])<<8);
	  
	  switch(vector_type)
	  {
		  case VECTOR_ACCELEROMETER:
			  bno055.vec_Data.vector.x = ((double)x) / 100.0;
			  bno055.vec_Data.vector.y = ((double)y) / 100.0;
			  bno055.vec_Data.vector.z = ((double)z) / 100.0;
			  bno055.vec_Data.updated =1;
			  break;
		  case VECTOR_MAGNETOMETER:
			  bno055.vec_Data.vector.x = ((double)x) / 16.0;
			  bno055.vec_Data.vector.y = ((double)y) / 16.0;
			  bno055.vec_Data.vector.z = ((double)z) / 16.0;
			  bno055.vec_Data.updated =1;
		  case VECTOR_GYROSCOPE:
			  bno055.vec_Data.vector.x = ((double)x) / 16.0;
			  bno055.vec_Data.vector.y = ((double)y) / 16.0;
			  bno055.vec_Data.vector.z = ((double)z) / 16.0;
			  bno055.vec_Data.updated =1;
			  break;
		  case VECTOR_EULER:
			  bno055.vec_Data.vector.x = ((double)x) / 16.0;
			  bno055.vec_Data.vector.y = ((double)y) / 16.0;
			  bno055.vec_Data.vector.z = ((double)z) / 16.0;
			  bno055.vec_Data.updated =1;
			  break;
		  case VECTOR_LINEARACCEL:
			  bno055.vec_Data.vector.x = ((double)x) / 100.0;
			  bno055.vec_Data.vector.y = ((double)y) / 100.0;
			  bno055.vec_Data.vector.z = ((double)z) / 100.0;
			  bno055.vec_Data.updated =1;
			  break;
		  case VECTOR_GRAVITY:
			  bno055.vec_Data.vector.x = ((double)x) / 100.0;
			  bno055.vec_Data.vector.y = ((double)y) / 100.0;
			  bno055.vec_Data.vector.z = ((double)z) / 100.0;
			  bno055.vec_Data.updated =1;
		  default : 
			  bno055.vec_Data.updated =0;
			  break;
	  }
  }
  