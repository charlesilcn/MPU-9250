#include<main.hpp>

int main()
int value;
wiringPiSetup(){
pinMode(8,INPUT); 
pinMode(9, OUTPUT); 
pinMode(2, INPUT);
pinMode(3,OUTPUT);
pinMode(27,INPUT);
pinMode(28 ,OUTPUT);
pinMode(4,OUTPUT);
pinMode(5,INPUT);
}
int MPU9250_SPI_Channel = 1;
const int MPU9250_ADDR = MPU9250_SIGNAL_PATH_RESET;
float _flag_MPU9250_LSB = 65.5;
int MPU9250_SPI_Freq = 1000000;