#include<main.hpp>
#include<iosteam>
int main() {
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
	  


	while (true)
	{
		std::cout << SF._uORB_Real_Pitch << "\n";
		std::cout << SF._uORB_Real__Roll << "\n";
	}


	








}


