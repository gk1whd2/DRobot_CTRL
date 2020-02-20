
#if !defined(DEFS_INCLUDE_)
#define DEFS_INCLUDE_

//#include "SBB_CANInit.h"
#include <stdint.h>


typedef struct {
	uint8_t sts_Limit[2];
	int16_t sts_Pos;
	int16_t cmd_PWR;
	int8_t	status_ENC;
}LINEAR_M;

typedef struct {
    int16_t sts_PWR;
    int16_t cmd_PWR;
}CATERPILLAR_M;


typedef struct{
    int16_t sts_rawData;
    int16_t sts_Distance;
		float 	sts_Distance_F;
    float   buff_rawData;
}PSD;

typedef struct{
    int16_t sts_rawData;
    int16_t sts_Current;
        float     sts_Current_F;
    float   buff_rawData;    
}INA1x8;
typedef struct{
		uint32_t  sts_rawData;
		float sts_Current_F;
}CURRENT_DRV8701;

    typedef struct{
    int16_t sts_Angle[3];
    int16_t sts_Acceleration[3];
    int16_t sts_Gyro[3];
}AHRS;
		

typedef struct{
		int8_t  stsCMD;
		int32_t cmdIndex;
		int32_t Value;
		int8_t  Flg_SW;
		int8_t  flgSW5;
		int8_t  flgSW6;
		int8_t  flgSW7;
		int16_t Limit_Body_H;
		int16_t Limit_Body_L;
		int16_t Limit_Seat_H;
}TILTING;


typedef struct{
    int16_t sts_rec_Vl;
    int16_t sts_rec_Vr;
    int16_t sts_rec_Lift_Tilt;
    int16_t sts_rec_Land_Tilt;
    int16_t sts_rec_Heartbeat;
    int16_t updateFlg_Cat;
    int16_t updateFlg_Linear;
}JoyData;

//typedef enum{
//    2Y0A21_F_FF = 0;
//    2Y0A21_F_03 = 1;
//}PSD_name;

typedef enum{
    axis_X = 0,
    axis_Y = 1,
    axis_Z = 2,
}axis_x;


typedef enum{
    LIFT_NONE = 0,
    LIFT_SEAT = 1,
    LIFT_BODY = 2,
}LIFT_x;


typedef enum{
    eulerAxis_PITCH = 0,
    eulerAxis_ROLL = 1,
    eulerAxis_YAW = 2,
}eulerAxis_x;



typedef enum{
    
    CAN_ID_DEV_LIFT_x01                = 0x01,
    
    CAN_ID_DEV_AHRS_ID_1         = 10,
    CAN_ID_DEV_AHRS_ID_2         = 11,
    
    CAN_ID_M2_POSITION_x90         = 0x90,
    CAN_ID_M2_SURF_DETECT_x91     = 0x91,
	  CAN_ID_M2_LIMITSWITCH_x92     = 0x92,
}CAN_ID_LIST;


typedef enum{
    MWIDEX_NONE                         = (int16_t)0,
    
    // MW User's Manual 106 Page
    MWIDEX_COMMAND_101             = 101,
    MWIDEX_POS_COMMAND_111     = 111,    // 0x6F
    MWIDEX_VEL_COMMAND_112     = 112,    // 0x70
    MWIDEX_CUR_COMMAND_113       = 113,
    MWIDEX_VOLT_COMMAND_114 = 114,

    // MW User's Manual 110 Page
    MWIDEX_STS_VOLTAGE_122    = 122,
    MWIDEX_STS_CURRENT_123    = 123,
    MWIDEX_STS_VELOCITY_124    = 124,
    MWIDEX_STS_POSITION_125    = 125,    // 0x7D
    MWIDEX_STS_HALL_CNT_126    = 126,
    // 테이터시트 참고하여 추가할 것 
}MW_INDEX_LIST;

// MW User's Manual 147 Page
typedef enum{
    // Access Code
    MW_CMD_ACC_CODE_WRITE_x10    = 0x10,
    MW_CMD_ACC_CODE_READ_x30    = 0x30,
    
    // Object Code
    MW_CMD_OBJ_CODE_I8_x00        = 0x00,
    MW_CMD_OBJ_CODE_I16_x04        = 0x04,
    MW_CMD_OBJ_CODE_I32_x08        = 0x08,
    MW_CMD_OBJ_CODE_FLOAT_x0c    = 0x0C,
    
    MW_CMD_WRITE_I32_x18        = 0x18,
    MW_CMD_READ_I32_x38            = 0x38,
    MW_CMD_READ_RES_I32_x48             = 0x48,
}MW_CANCMD_MASK;

typedef enum{
    PID_VEL_CMD_130 = 130,
    // 테이터시트 참고하여 추가할 것 
}PID_VEL_LIST;


typedef enum{
    lift_Rising_NONE = 0,
    lift_Rising_Start = 10,
    lift_Rising_Flat ,
    lift_Rising_End,
    lift_Falling_NONE = 0,
    lift_Falling_Start = 0,
    lift_Falling_Flat,
    lift_Falling_End,
}lift_Rising_Sts;

typedef enum{
		stsWrite 		= 2,
		stsRead  		= 1,
		CMD_Degree 	= 200,
		CMD_Kp 			= 210,
		CMD_Pwr 		= 201,
		S_INDEX_BODY= 0,
		S_INDEX_SEAT= 1,
}Tilting_Sts;

typedef struct 
{
    int16_t raw_acc[3];
    int16_t raw_gyro[3];
    int16_t raw_euler[3];
    int16_t raw_mag[3];
    
    int16_t raw_euler_Offset[3];
    
    float conv_acc[3];
    float conv_gyro[3];
    float conv_euler[3];
    float conv_mag[3];
    
    uint16_t updateFlg;
    uint16_t updatedTime;
}NTAHRS_DATA;


typedef enum
{
	LDC_STATE_NONE 						=		 	0,
	LDC_STATE_READING				=			1,
	LDC_STATE_COMPLETE		=			2,
}LDC_STATE;

typedef enum
{
	LDC_SCALE_64			=		27,
	LDC_SCALE_128		=		25,
}LDC_SCALE;

typedef struct
{
	int32_t ldc_data[2];
	LDC_SCALE ldc_scale;
	LDC_STATE ldc_state;
}LOADCELL;

#endif
