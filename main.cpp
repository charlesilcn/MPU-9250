#include<iostream>
#include<WiringPiI2C.h>
#include<math.h>
#include<stdio.h>
using namespace std;
int a;
int MPU9250_ADDR;
int V;
int XRollAngle;
int YRollAngle;
int main()
{
	a = wiringPiI2CSetup(MPU9250_ADDR);

	wiringPiI2CWriteReg8(a, 107, 0x00);
	wiringPiI2CWriteReg8(a, 28, 0x08);
	wiringPiI2CWriteReg8(a, 27, 0x08);
	wiringPiI2CWriteReg8(a, 26, 0x03);

	int XH = wiringPiI2CReadReg8(a, 59);
	int XL = wiringPiI2CReadReg8(a, 60);
	int YH = wiringPiI2CReadReg8(a, 61);
	int YL = wiringPiI2CReadReg8(a, 62);
	int ZH = wiringPiI2CReadReg8(a, 63);
	int ZL = wiringPiI2CReadReg8(a, 64);

	int X = XH * 255 + XL;
	int Y = YH * 255 + YL;
	int Z = ZH * 255 + ZL;
	V = sqrt((X * X) + (Y * Y) + (Z * Z));
	XRollAngle = asin((float)X / V) * -57.296;
	YRollAngle = asin((float)Y / V) * 57.296;
	std::cout << XRollAngle << "\n";
	std::cout << YRollAngle << "\n";
