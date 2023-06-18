#pragma once
#ifndef IMU_H

#define omega_e 7.292115e-5
#define _a 6378137.0
#define _b 6356752.31425
#define GM 3.986005e14
#define e2 0.00669437999013
#define _alpha (1/298.257223563)
#define dataRate 100.0 //HZ
#define toa 300.0 // timeOfAlignment   s


#include<iostream>
#include<string.h>
#include<stdio.h>
#include<map>
#include<ostream>
#include<fstream>

#include"Attitude.h"
#include"Coor.h"
#include"Matrix.h"
#include"Time.h"
#include"Vector.h"

struct _IMU
{
	GPST gt;
	double dt;
	double accl[3];//m/s
	double gyro[3];//弧度rad
	Vector dvel;
	Vector dtheta;
	_IMU()
	{
		dt = 0.0;
		for (int i = 0; i < 3; i++)
		{
			gyro[i] = accl[i] = 0.0;
		}
		dtheta = gyro;
		dvel = accl;
	}
};

struct _IMUError
{
	Vector bg;
	Vector ba;
	Vector Sg;
	Vector Sa;
	_IMUError()
	{
		double array[3] = { 0.0 };
		bg = array;
		ba = array;
		Sg = array;
		Sa = array;
	}
};

void ReadIMUData(char* FileName, std::map<double, _IMU> &mapImu);
std::ostream& operator<<(std::ostream& out, _IMU& imu);
std::ofstream& operator<<(std::ofstream& out, _IMU& imu);
std::ofstream& operator<<(std::ofstream& out, _IMUError& imuError);

/*IMU数据补偿*/
void imuCompensate(_IMU& imu, _IMUError& imuerror);
/*内插*/
void imuInterpolate(_IMU& imu1, _IMU& imu2, double updatetime, _IMU& imumid);


#endif // !IMU_H
