#include<iostream>
#include<string>
#include<unistd.h>
#include<thread>
#include<wiringPiSPI.h>
#include<math.h>
#include<fstream>
#define MPUIsI2c 0
#define MPUIsSpi 1


inline void IMUSensorsDataRead()
{
	if (SF.MPU9250_Type == MPUIsI2c)
	{
		SF._Tmp_MPU9250_Buffer[0] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3B);
		SF._Tmp_MPU9250_Buffer[1] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3C);
		SF._Tmp_MPU9250_A_X = (SF._Tmp_MPU9250_Buffer[0] << 8 | SF._Tmp_MPU9250_Buffer[1]);
		SF._uORB_MPU9250_A_X = (short)SF._Tmp_MPU9250_A_X;
		SF._Tmp_MPU9250_Buffer[2] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3D);
		SF._Tmp_MPU9250_Buffer[3] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3E);
		SF._Tmp_MPU9250_A_Y = (SF._Tmp_MPU9250_Buffer[2] << 8 | SF._Tmp_MPU9250_Buffer[3]);
		SF._uORB_MPU9250_A_Y = (short)SF._Tmp_MPU9250_A_Y;
		SF._Tmp_MPU9250_Buffer[4] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x3F);
		SF._Tmp_MPU9250_Buffer[5] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x40);
		SF._Tmp_MPU9250_A_Z = (SF._Tmp_MPU9250_Buffer[4] << 8 | SF._Tmp_MPU9250_Buffer[5]);
		SF._uORB_MPU9250_A_Z = (short)SF._Tmp_MPU9250_A_Z;

		SF._Tmp_MPU9250_Buffer[6] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x43);
		SF._Tmp_MPU9250_Buffer[7] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x44);
		SF._Tmp_MPU9250_G_X = (SF._Tmp_MPU9250_Buffer[6] << 8 | SF._Tmp_MPU9250_Buffer[7]);
		SF._uORB_MPU9250_G_X = (short)SF._Tmp_MPU9250_G_X;
		SF._Tmp_MPU9250_Buffer[8] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x45);
		SF._Tmp_MPU9250_Buffer[9] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x46);
		SF._Tmp_MPU9250_G_Y = (SF._Tmp_MPU9250_Buffer[8] << 8 | SF._Tmp_MPU9250_Buffer[9]);
		SF._uORB_MPU9250_G_Y = (short)SF._Tmp_MPU9250_G_Y;
		SF._Tmp_MPU9250_Buffer[10] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x47);
		SF._Tmp_MPU9250_Buffer[11] = wiringPiI2CReadReg8(DF.MPU9250_fd, 0x48);
		SF._Tmp_MPU9250_G_Z = (SF._Tmp_MPU9250_Buffer[10] << 8 | SF._Tmp_MPU9250_Buffer[11]);
		SF._uORB_MPU9250_G_Z = (short)SF._Tmp_MPU9250_G_Z;
	}
	else if (SF.MPU9250_Type == MPUIsSpi)
	{
		SF._Tmp_MPU9250_SPI_Buffer[0] = 0xBB;
		wiringPiSPIDataRW(1, SF._Tmp_MPU9250_SPI_Buffer, 20);
		SF._Tmp_MPU9250_A_X = ((int)SF._Tmp_MPU9250_SPI_Buffer[1] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[2]);
		SF._uORB_MPU9250_A_X = (short)SF._Tmp_MPU9250_A_X;
		SF._Tmp_MPU9250_A_Y = ((int)SF._Tmp_MPU9250_SPI_Buffer[3] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[4]);
		SF._uORB_MPU9250_A_Y = (short)SF._Tmp_MPU9250_A_Y;
		SF._Tmp_MPU9250_A_Z = ((int)SF._Tmp_MPU9250_SPI_Buffer[5] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[6]);
		SF._uORB_MPU9250_A_Z = (short)SF._Tmp_MPU9250_A_Z;

		SF._Tmp_MPU9250_G_X = ((int)SF._Tmp_MPU9250_SPI_Buffer[9] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[10]);
		SF._uORB_MPU9250_G_X = (short)SF._Tmp_MPU9250_G_X;
		SF._Tmp_MPU9250_G_Y = ((int)SF._Tmp_MPU9250_SPI_Buffer[11] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[12]);
		SF._uORB_MPU9250_G_Y = (short)SF._Tmp_MPU9250_G_Y;
		SF._Tmp_MPU9250_G_Z = ((int)SF._Tmp_MPU9250_SPI_Buffer[13] << 8 | (int)SF._Tmp_MPU9250_SPI_Buffer[14]);
		SF._uORB_MPU9250_G_Z = (short)SF._Tmp_MPU9250_G_Z;
	}
}
struct SensorsINFO
{
	//=========================MPU9250======//
	int MPU9250_Type;
	int _Tmp_MPU9250_Buffer[14];
	unsigned char _Tmp_MPU9250_SPI_Config[5];
	unsigned char _Tmp_MPU9250_SPI_Buffer[28];

	long _uORB_MPU9250_A_X;
	long _uORB_MPU9250_A_Y;
	long _uORB_MPU9250_A_Z;
	long _uORB_MPU9250_G_X;
	long _uORB_MPU9250_G_Y;
	long _uORB_MPU9250_G_Z;

	float _uORB_Accel__Roll = 0;
	float _uORB_Accel_Pitch = 0;
	float _uORB_Gryo__Roll = 0;
	float _uORB_Gryo_Pitch = 0;
	float _uORB_Gryo___Yaw = 0;
	float _uORB_Real_Pitch = 0;
	float _uORB_Real__Roll = 0;

	unsigned long _Tmp_MPU9250_G_X;
	unsigned long _Tmp_MPU9250_G_Y;
	unsigned long _Tmp_MPU9250_G_Z;
	unsigned long _Tmp_MPU9250_A_X;
	unsigned long _Tmp_MPU9250_A_Y;
	unsigned long _Tmp_MPU9250_A_Z;

	long _flag_MPU9250_G_X_Cali;
	long _flag_MPU9250_G_Y_Cali;
	long _flag_MPU9250_G_Z_Cali;
	double _flag_Accel__Roll_Cali;
	double _flag_Accel_Pitch_Cali;

	long _Tmp_IMU_Accel_Calibration[20];
	long _Tmp_IMU_Accel_Vector;

	long _Tmp_Gryo_filer_Input_Quene_X[3] = { 0 , 0 ,0 };
	long _Tmp_Gryo_filer_Output_Quene_X[3] = { 0 , 0 ,0 };

	long _Tmp_Gryo_filer_Input_Quene_Y[3] = { 0 , 0 ,0 };
	long _Tmp_Gryo_filer_Output_Quene_Y[3] = { 0 , 0 ,0 };

	long _Tmp_Gryo_filer_Input_Quene_Z[3] = { 0 , 0 ,0 };
	long _Tmp_Gryo_filer_Output_Quene_Z[3] = { 0 , 0 ,0 };
	float _flag_Filter_Gain = 4.840925170e+00;
	
}SF;

inline void SensorsParse()
{
	AF.Update_TimerStart = micros();
	IMUSensorsDataRead();
	//Gryo----------------------------------------------------------------------//
	SF._uORB_MPU9250_G_X -= SF._flag_MPU9250_G_X_Cali;
	SF._uORB_MPU9250_G_Y -= SF._flag_MPU9250_G_Y_Cali;
	SF._uORB_MPU9250_G_Z -= SF._flag_MPU9250_G_Z_Cali;
	IMUGryoFilter(SF._uORB_MPU9250_G_X, SF._uORB_MPU9250_G_X, SF._Tmp_Gryo_filer_Input_Quene_X, SF._Tmp_Gryo_filer_Output_Quene_X);
	IMUGryoFilter(SF._uORB_MPU9250_G_Y, SF._uORB_MPU9250_G_Y, SF._Tmp_Gryo_filer_Input_Quene_Y, SF._Tmp_Gryo_filer_Output_Quene_Y);
	IMUGryoFilter(SF._uORB_MPU9250_G_Z, SF._uORB_MPU9250_G_Z, SF._Tmp_Gryo_filer_Input_Quene_Z, SF._Tmp_Gryo_filer_Output_Quene_Z);
	SF._uORB_Gryo__Roll = (SF._uORB_Gryo__Roll * 0.7) + ((SF._uORB_MPU9250_G_Y / DF._flag_MPU9250_LSB) * 0.3);
	SF._uORB_Gryo_Pitch = (SF._uORB_Gryo_Pitch * 0.7) + ((SF._uORB_MPU9250_G_X / DF._flag_MPU9250_LSB) * 0.3);
	SF._uORB_Gryo___Yaw = (SF._uORB_Gryo___Yaw * 0.7) + ((SF._uORB_MPU9250_G_Z / DF._flag_MPU9250_LSB) * 0.3);
	//ACCEL---------------------------------------------------------------------//
	SF._Tmp_IMU_Accel_Vector = sqrt((SF._uORB_MPU9250_A_X * SF._uORB_MPU9250_A_X) + (SF._uORB_MPU9250_A_Y * SF._uORB_MPU9250_A_Y) + (SF._uORB_MPU9250_A_Z * SF._uORB_MPU9250_A_Z));
	if (abs(SF._uORB_MPU9250_A_X) < SF._Tmp_IMU_Accel_Vector)
		SF._uORB_Accel__Roll = asin((float)SF._uORB_MPU9250_A_X / SF._Tmp_IMU_Accel_Vector) * -57.296;
	if (abs(SF._uORB_MPU9250_A_Y) < SF._Tmp_IMU_Accel_Vector)
		SF._uORB_Accel_Pitch = asin((float)SF._uORB_MPU9250_A_Y / SF._Tmp_IMU_Accel_Vector) * 57.296;
	SF._uORB_Accel__Roll -= SF._flag_Accel__Roll_Cali;
	SF._uORB_Accel_Pitch -= SF._flag_Accel_Pitch_Cali;
	//Gryo_MIX_ACCEL------------------------------------------------------------//
	SF._uORB_Real_Pitch += SF._uORB_MPU9250_G_X / AF.Update_Freqeuncy / DF._flag_MPU9250_LSB;
	SF._uORB_Real__Roll += SF._uORB_MPU9250_G_Y / AF.Update_Freqeuncy / DF._flag_MPU9250_LSB;
	SF._uORB_Real_Pitch -= SF._uORB_Real__Roll * sin((SF._uORB_MPU9250_G_Z / AF.Update_Freqeuncy / DF._flag_MPU9250_LSB) * (3.14 / 180));
	SF._uORB_Real__Roll += SF._uORB_Real_Pitch * sin((SF._uORB_MPU9250_G_Z / AF.Update_Freqeuncy / DF._flag_MPU9250_LSB) * (3.14 / 180));
	if (!AF._flag_first_StartUp)
	{
		SF._uORB_Real_Pitch = SF._uORB_Real_Pitch * 0.9994 + SF._uORB_Accel_Pitch * 0.0006;
		SF._uORB_Real__Roll = SF._uORB_Real__Roll * 0.9994 + SF._uORB_Accel__Roll * 0.0006;
	}
	else
	{
		SF._uORB_Real_Pitch = SF._uORB_Accel_Pitch;
		SF._uORB_Real__Roll = SF._uORB_Accel__Roll;
		AF._flag_first_StartUp = false;
	}
}











/*
#define MPU9250_DEFAULT_ADDRESS         0xD1
#define MPU9250_ALT_DEFAULT_ADDRESS     0xD2   

#define MPU9250_SELF_TEST_X_GYRO        0x00
#define MPU9250_SELF_TEST_Y_GYRO        0x01
#define MPU9250_SELF_TEST_Z_GYRO        0x02

#define MPU9250_SELF_TEST_X_ACCEL       0x0D
#define MPU9250_SELF_TEST_Y_ACCEL       0x0E
#define MPU9250_SELF_TEST_Z_ACCEL       0x0F

#define MPU9250_XG_OFFSET_H             0x13
#define MPU9250_XG_OFFSET_L             0x14
#define MPU9250_YG_OFFSET_H             0x15
#define MPU9250_YG_OFFSET_L             0x16
#define MPU9250_ZG_OFFSET_H             0x17
#define MPU9250_ZG_OFFSET_L             0x18
#define MPU9250_SMPLRT_DIV              0x19
#define MPU9250_CONFIG                  0x1A
#define MPU9250_GYRO_CONFIG             0x1B
#define MPU9250_ACCEL_CONFIG            0x1C
#define MPU9250_ACCEL_CONFIG2           0x1D
#define MPU9250_LP_ACCEL_ODR            0x1E
#define MPU9250_WOM_THR                 0x1F

#define MPU9250_FIFO_EN                 0x23
#define MPU9250_I2C_MST_CTRL            0x24
#define MPU9250_I2C_SLV0_ADDR           0x25
#define MPU9250_I2C_SLV0_REG            0x26
#define MPU9250_I2C_SLV0_CTRL           0x27
#define MPU9250_I2C_SLV1_ADDR           0x28
#define MPU9250_I2C_SLV1_REG            0x29
#define MPU9250_I2C_SLV1_CTRL           0x2A
#define MPU9250_I2C_SLV2_ADDR           0x2B
#define MPU9250_I2C_SLV2_REG            0x2C
#define MPU9250_I2C_SLV2_CTRL           0x2D
#define MPU9250_I2C_SLV3_ADDR           0x2E
#define MPU9250_I2C_SLV3_REG            0x2F
#define MPU9250_I2C_SLV3_CTRL           0x30
#define MPU9250_I2C_SLV4_ADDR           0x31
#define MPU9250_I2C_SLV4_REG            0x32
#define MPU9250_I2C_SLV4_DO             0x33
#define MPU9250_I2C_SLV4_CTRL           0x34
#define MPU9250_I2C_SLV4_DI             0x35
#define MPU9250_I2C_MST_STATUS          0x36
#define MPU9250_INT_PIN_CFG             0x37
#define MPU9250_INT_ENABLE              0x38

#define MPU9250_INT_STATUS              0x3A
#define MPU9250_ACCEL_XOUT_H            0x3B
#define MPU9250_ACCEL_XOUT_L            0x3C
#define MPU9250_ACCEL_YOUT_H            0x3D
#define MPU9250_ACCEL_YOUT_L            0x3E
#define MPU9250_ACCEL_ZOUT_H            0x3F
#define MPU9250_ACCEL_ZOUT_L            0x40
#define MPU9250_TEMP_OUT_H              0x41
#define MPU9250_TEMP_OUT_L              0x42
#define MPU9250_GYRO_XOUT_H             0x43
#define MPU9250_GYRO_XOUT_L             0x44
#define MPU9250_GYRO_YOUT_H             0x45
#define MPU9250_GYRO_YOUT_L             0x46
#define MPU9250_GYRO_ZOUT_H             0x47
#define MPU9250_GYRO_ZOUT_L             0x48
#define MPU9250_EXT_SENS_DATA_00        0x49
#define MPU9250_EXT_SENS_DATA_01        0x4A
#define MPU9250_EXT_SENS_DATA_02        0x4B
#define MPU9250_EXT_SENS_DATA_03        0x4C
#define MPU9250_EXT_SENS_DATA_04        0x4D
#define MPU9250_EXT_SENS_DATA_05        0x4E
#define MPU9250_EXT_SENS_DATA_06        0x4F
#define MPU9250_EXT_SENS_DATA_07        0x50
#define MPU9250_EXT_SENS_DATA_08        0x51
#define MPU9250_EXT_SENS_DATA_09        0x52
#define MPU9250_EXT_SENS_DATA_10        0x53
#define MPU9250_EXT_SENS_DATA_11        0x54
#define MPU9250_EXT_SENS_DATA_12        0x55
#define MPU9250_EXT_SENS_DATA_13        0x56
#define MPU9250_EXT_SENS_DATA_14        0x57
#define MPU9250_EXT_SENS_DATA_15        0x58
#define MPU9250_EXT_SENS_DATA_16        0x59
#define MPU9250_EXT_SENS_DATA_17        0x5A
#define MPU9250_EXT_SENS_DATA_18        0x5B
#define MPU9250_EXT_SENS_DATA_19        0x5C
#define MPU9250_EXT_SENS_DATA_20        0x5D
#define MPU9250_EXT_SENS_DATA_21        0x5E
#define MPU9250_EXT_SENS_DATA_22        0x5F
#define MPU9250_EXT_SENS_DATA_23        0x60

#define MPU9250_I2C_SLV0_DO             0x63
#define MPU9250_I2C_SLV1_DO             0x64
#define MPU9250_I2C_SLV2_DO             0x65
#define MPU9250_I2C_SLV3_DO             0x66
#define MPU9250_I2C_MST_DELAY_CTRL      0x67
#define MPU9250_SIGNAL_PATH_RESET       0x68
#define MPU9250_MOT_DETECT_CTRL         0x69
#define MPU9250_USER_CTRL               0x6A
#define MPU9250_PWR_MGMT_1              0x6B
#define MPU9250_PWR_MGMT_2              0x6C

#define MPU9250_FIFO_COUNTH             0x72
#define MPU9250_FIFO_COUNTL             0x73
#define MPU9250_FIFO_R_W                0x74
#define MPU9250_WHO_AM_I                0x75
#define MPU9250_XA_OFFSET_H             0x77
#define MPU9250_XA_OFFSET_L             0x78

#define MPU9250_YA_OFFSET_H             0x7A
#define MPU9250_YA_OFFSET_L             0x7B

#define MPU9250_ZA_OFFSET_H             0x7D
#define MPU9250_ZA_OFFSET_L             0x7E

//reset values
#define WHOAMI_RESET_VAL                0x71
#define POWER_MANAGMENT_1_RESET_VAL     0x01
#define DEFAULT_RESET_VALUE             0x00

#define WHOAMI_DEFAULT_VAL              0x68

//CONFIG register masks
#define MPU9250_FIFO_MODE_MASK          0x40
#define MPU9250_EXT_SYNC_SET_MASK       0x38
#define MPU9250_DLPF_CFG_MASK           0x07

//GYRO_CONFIG register masks
#define MPU9250_XGYRO_CTEN_MASK         0x80
#define MPU9250_YGYRO_CTEN_MASK         0x40
#define MPU9250_ZGYRO_CTEN_MASK         0x20
#define MPU9250_GYRO_FS_SEL_MASK        0x18
#define MPU9250_FCHOICE_B_MASK          0x03

#define MPU9250_GYRO_FULL_SCALE_250DPS  0
#define MPU9250_GYRO_FULL_SCALE_500DPS  1
#define MPU9250_GYRO_FULL_SCALE_1000DPS 2
#define MPU9250_GYRO_FULL_SCALE_2000DPS 3

//ACCEL_CONFIG register masks
#define MPU9250_AX_ST_EN_MASK           0x80
#define MPU9250_AY_ST_EN_MASK           0x40
#define MPU9250_AZ_ST_EN_MASK           0x20
#define MPU9250_ACCEL_FS_SEL_MASK       0x18

#define MPU9250_FULL_SCALE_2G           0
#define MPU9250_FULL_SCALE_4G           1
#define MPU9250_FULL_SCALE_8G           2
#define MPU9250_FULL_SCALE_16G          3

//ACCEL_CONFIG_2 register masks
#define MPU9250_ACCEL_FCHOICE_B_MASK    0xC0
#define MPU9250_A_DLPF_CFG_MASK         0x03

//LP_ACCEL_ODR register masks
#define MPU9250_LPOSC_CLKSEL_MASK       0x0F

//FIFO_EN register masks
#define MPU9250_TEMP_FIFO_EN_MASK       0x80
#define MPU9250_GYRO_XOUT_MASK          0x40
#define MPU9250_GYRO_YOUT_MASK          0x20
#define MPU9250_GYRO_ZOUT_MASK          0x10
#define MPU9250_ACCEL_MASK              0x08
#define MPU9250_SLV2_MASK               0x04
#define MPU9250_SLV1_MASK               0x02
#define MPU9250_SLV0_MASK               0x01

//I2C_MST_CTRL register masks
#define MPU9250_MULT_MST_EN_MASK        0x80
#define MPU9250_WAIT_FOR_ES_MASK        0x40
#define MPU9250_SLV_3_FIFO_EN_MASK      0x20
#define MPU9250_I2C_MST_P_NSR_MASK      0x10
#define MPU9250_I2C_MST_CLK_MASK        0x0F

//I2C_SLV0_ADDR register masks
#define MPU9250_I2C_SLV0_RNW_MASK       0x80
#define MPU9250_I2C_ID_0_MASK           0x7F

//I2C_SLV0_CTRL register masks
#define MPU9250_I2C_SLV0_EN_MASK        0x80
#define MPU9250_I2C_SLV0_BYTE_SW_MASK   0x40
#define MPU9250_I2C_SLV0_REG_DIS_MASK   0x20
#define MPU9250_I2C_SLV0_GRP_MASK       0x10
#define MPU9250_I2C_SLV0_LENG_MASK      0x0F

//I2C_SLV1_ADDR register masks
#define MPU9250_I2C_SLV1_RNW_MASK       0x80
#define MPU9250_I2C_ID_1_MASK           0x7F

//I2C_SLV1_CTRL register masks
#define MPU9250_I2C_SLV1_EN_MASK        0x80
#define MPU9250_I2C_SLV1_BYTE_SW_MASK   0x40
#define MPU9250_I2C_SLV1_REG_DIS_MASK   0x20
#define MPU9250_I2C_SLV1_GRP_MASK       0x10
#define MPU9250_I2C_SLV1_LENG_MASK      0x0F

//I2C_SLV2_ADDR register masks
#define MPU9250_I2C_SLV2_RNW_MASK       0x80
#define MPU9250_I2C_ID_2_MASK           0x7F

//I2C_SLV2_CTRL register masks
#define MPU9250_I2C_SLV2_EN_MASK        0x80
#define MPU9250_I2C_SLV2_BYTE_SW_MASK   0x40
#define MPU9250_I2C_SLV2_REG_DIS_MASK   0x20
#define MPU9250_I2C_SLV2_GRP_MASK       0x10
#define MPU9250_I2C_SLV2_LENG_MASK      0x0F

//I2C_SLV3_ADDR register masks
#define MPU9250_I2C_SLV3_RNW_MASK       0x80
#define MPU9250_I2C_ID_3_MASK           0x7F

//I2C_SLV3_CTRL register masks
#define MPU9250_I2C_SLV3_EN_MASK        0x80
#define MPU9250_I2C_SLV3_BYTE_SW_MASK   0x40
#define MPU9250_I2C_SLV3_REG_DIS_MASK   0x20
#define MPU9250_I2C_SLV3_GRP_MASK       0x10
#define MPU9250_I2C_SLV3_LENG_MASK      0x0F

//I2C_SLV4_ADDR register masks
#define MPU9250_I2C_SLV4_RNW_MASK       0x80
#define MPU9250_I2C_ID_4_MASK           0x7F

//I2C_SLV4_CTRL register masks
#define MPU9250_I2C_SLV4_EN_MASK        0x80
#define MPU9250_SLV4_DONE_INT_EN_MASK   0x40
#define MPU9250_I2C_SLV4_REG_DIS_MASK   0x20
#define MPU9250_I2C_MST_DLY_MASK        0x1F

//I2C_MST_STATUS register masks
#define MPU9250_PASS_THROUGH_MASK       0x80
#define MPU9250_I2C_SLV4_DONE_MASK      0x40
#define MPU9250_I2C_LOST_ARB_MASK       0x20
#define MPU9250_I2C_SLV4_NACK_MASK      0x10
#define MPU9250_I2C_SLV3_NACK_MASK      0x08
#define MPU9250_I2C_SLV2_NACK_MASK      0x04
#define MPU9250_I2C_SLV1_NACK_MASK      0x02
#define MPU9250_I2C_SLV0_NACK_MASK      0x01

//INT_PIN_CFG register masks
#define MPU9250_ACTL_MASK               0x80
#define MPU9250_OPEN_MASK               0x40
#define MPU9250_LATCH_INT_EN_MASK       0x20
#define MPU9250_INT_ANYRD_2CLEAR_MASK   0x10
#define MPU9250_ACTL_FSYNC_MASK         0x08
#define MPU9250_FSYNC_INT_MODE_EN_MASK  0x04
#define MPU9250_BYPASS_EN_MASK          0x02

//INT_ENABLE register masks
#define MPU9250_WOM_EN_MASK             0x40
#define MPU9250_FIFO_OFLOW_EN_MASK      0x10
#define MPU9250_FSYNC_INT_EN_MASK       0x08
#define MPU9250_RAW_RDY_EN_MASK         0x01

//INT_STATUS register masks
#define MPU9250_WOM_INT_MASK            0x40
#define MPU9250_FIFO_OFLOW_INT_MASK     0x10
#define MPU9250_FSYNC_INT_MASK          0x08
#define MPU9250_RAW_DATA_RDY_INT_MASK   0x01

//I2C_MST_DELAY_CTRL register masks
#define MPU9250_DELAY_ES_SHADOW_MASK    0x80
#define MPU9250_I2C_SLV4_DLY_EN_MASK    0x10
#define MPU9250_I2C_SLV3_DLY_EN_MASK    0x08
#define MPU9250_I2C_SLV2_DLY_EN_MASK    0x04
#define MPU9250_I2C_SLV1_DLY_EN_MASK    0x02
#define MPU9250_I2C_SLV0_DLY_EN_MASK    0x01

//SIGNAL_PATH_RESET register masks
#define MPU9250_GYRO_RST_MASK           0x04
#define MPU9250_ACCEL_RST_MASK          0x02
#define MPU9250_TEMP_RST_MASK           0x01

//MOT_DETECT_CTRL register masks
#define MPU9250_ACCEL_INTEL_EN_MASK     0x80
#define MPU9250_ACCEL_INTEL_MODE_MASK   0x40

//USER_CTRL register masks
#define MPU9250_FIFO_EN_MASK            0x40
#define MPU9250_I2C_MST_EN_MASK         0x20
#define MPU9250_I2C_IF_DIS_MASK         0x10
#define MPU9250_FIFO_RST_MASK           0x04
#define MPU9250_I2C_MST_RST_MASK        0x02
#define MPU9250_SIG_COND_RST_MASK       0x01

//PWR_MGMT_1 register masks
#define MPU9250_H_RESET_MASK            0x80
#define MPU9250_SLEEP_MASK              0x40
#define MPU9250_CYCLE_MASK              0x20
#define MPU9250_GYRO_STANDBY_CYCLE_MASK 0x10
#define MPU9250_PD_PTAT_MASK            0x08
#define MPU9250_CLKSEL_MASK             0x07

//PWR_MGMT_2 register masks
#define MPU9250_DISABLE_XA_MASK         0x20
#define MPU9250_DISABLE_YA_MASK         0x10
#define MPU9250_DISABLE_ZA_MASK         0x08
#define MPU9250_DISABLE_XG_MASK         0x04
#define MPU9250_DISABLE_YG_MASK         0x02
#define MPU9250_DISABLE_ZG_MASK         0x01

#define MPU9250_DISABLE_XYZA_MASK       0x38
#define MPU9250_DISABLE_XYZG_MASK       0x07

//Magnetometer register maps
#define MPU9250_MAG_ADDRESS             0x0C

#define MPU9250_MAG_WIA                 0x00
#define MPU9250_MAG_INFO                0x01
#define MPU9250_MAG_ST1                 0x02
#define MPU9250_MAG_XOUT_L              0x03
#define MPU9250_MAG_XOUT_H              0x04
#define MPU9250_MAG_YOUT_L              0x05
#define MPU9250_MAG_YOUT_H              0x06
#define MPU9250_MAG_ZOUT_L              0x07
#define MPU9250_MAG_ZOUT_H              0x08
#define MPU9250_MAG_ST2                 0x09
#define MPU9250_MAG_CNTL                0x0A
#define MPU9250_MAG_RSV                 0x0B 
#define MPU9250_MAG_ASTC                0x0C
#define MPU9250_MAG_TS1                 0x0D
#define MPU9250_MAG_TS2                 0x0E
#define MPU9250_MAG_I2CDIS              0x0F
#define MPU9250_MAG_ASAX                0x10
#define MPU9250_MAG_ASAY                0x11
#define MPU9250_MAG_ASAZ                0x12

int MPU9250_SPI_Channel = 1;
const int MPU9250_ADDR = MPU9250_SIGNAL_PATH_RESET;
float _flag_MPU9250_LSB = 65.5;
int MPU9250_SPI_Freq = 1000000;
bool _flag_first_StartUp = true;

long _uORB_MPU9250_A_X;
long _uORB_MPU9250_A_Y;
long _uORB_MPU9250_A_Z;

long _uORB_MPU9250_G_X;
long _uORB_MPU9250_G_Y;
long _uORB_MPU9250_G_Z;

long _Flag_MPU9250_G_X_Cali = 0;
long _Flag_MPU9250_G_Y_Cali = 0;
long _Flag_MPU9250_G_Z_Cali = 0;

long _Flag_MPU9250_A_X_Cali = 0;
long _Flag_MPU9250_A_Y_Cali = 0;

#ifdef SPI_MPU9250
MPU9250_fd = wiringPiSPISetup(MPU9250_SPI_Channel, MPU9250_SPI_Freq);
if (MPU9250_fd < 0)
	Status_Code[2] = -1;
else
{
	_Tmp_MPU9250_SPI_Config[0] = MPU9250_PWR_MGMT_1;
	_Tmp_MPU9250_SPI_Config[1] = MPU9250_PWR_MGMT_1;
	wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Config, 2); //reset
	_Tmp_MPU9250_SPI_Config[0] = MPU9250_ACCEL_CONFIG;
	_Tmp_MPU9250_SPI_Config[1] = MPU9250_MAG_ZOUT_H;
	wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Config, 2); // Accel
	_Tmp_MPU9250_SPI_Config[0] = MPU9250_GYRO_CONFIG;
	_Tmp_MPU9250_SPI_Config[1] = MPU9250_MAG_ZOUT_H;
	wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Config, 2); // Gryo
	_Tmp_MPU9250_SPI_Config[0] = MPU9250_CONFIG;
	_Tmp_MPU9250_SPI_Config[1] = MPU9250_MAG_XOUT_L;
	wiringPiSPIDataRW(1, _Tmp_MPU9250_SPI_Config, 2); //config
	Status_Code[2] = 0;
}
