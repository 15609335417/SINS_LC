#include "IMU.h"

using namespace std;

void ReadIMUData(char* FileName, std::map<double, _IMU> &mapImu)
{
	FILE* fp_ins;
	_IMU imu;
	_IMU imupre;
	char buffer[10240];
	char* str;
	const char* d = " ,;*";
	char* ptr = NULL;
	double acc_scale = 0.05 / pow(2, 15);
	double gyo_scale = 0.1 / 3600.0 / pow(2, 8);
	fopen_s(&fp_ins, FileName, "r+b");
	if (fp_ins == 0)
	{
		printf("Open Error!");
		return;
	}
	while (feof(fp_ins) == 0)
	{
		fgets(buffer, 10240, fp_ins);
		if (*buffer != '%') continue;
		str = strtok_s(buffer, d, &ptr); //printf("%s\n", str);    //% RAWIMUSA
		if (str != NULL)
		{
			str = strtok_s(NULL, d, &ptr); if (str != NULL) imu.gt.Week = atoi(str); //printf("%s\n", str);        //GPST week
			str = strtok_s(NULL, d, &ptr); if (str != NULL) imu.gt.SecOfWeek =atof(str); //printf("%s\n", str);   //GPST sow
			str = strtok_s(NULL, d, &ptr); //printf("%s\n", str);
			str = strtok_s(NULL, d, &ptr); //printf("%s\n", str);
			str = strtok_s(NULL, d, &ptr); //printf("%s\n", str);
			//前右上->右前下
			str = strtok_s(NULL, d, &ptr); if (str != NULL)imu.accl[2] = atof(str) * acc_scale * (-1); //printf("%s\n", str);//acc_Z
			str = strtok_s(NULL, d, &ptr); if (str != NULL)imu.accl[0] = atof(str) * acc_scale * (-1); //printf("%s\n", str);//acc_Y
			str = strtok_s(NULL, d, &ptr); if (str != NULL)imu.accl[1] = atof(str) * acc_scale; //printf("%s\n", str);       //acc_X
			str = strtok_s(NULL, d, &ptr); if (str != NULL)imu.gyro[2] = atof(str) * gyo_scale * (-1); //printf("%s\n", str);//gyo_Z
			str = strtok_s(NULL, d, &ptr); if (str != NULL)imu.gyro[0] = atof(str) * gyo_scale * (-1); //printf("%s\n", str);//gyo_Y
			str = strtok_s(NULL, d, &ptr); if (str != NULL)imu.gyro[1] = atof(str) * gyo_scale; //printf("%s\n", str);       //gyo_X
			str = strtok_s(NULL, d, &ptr); //printf("%s\n", str);
		}
		imu.dtheta = imu.gyro;
		imu.dvel = imu.accl;
		//缺失数据插值
		while (imu.gt.SecOfWeek - imupre.gt.SecOfWeek - 1.0 / dataRate > EPS && imupre.gt.SecOfWeek > EPS)
		{
			int num = (imu.gt.SecOfWeek - imupre.gt.SecOfWeek) / (1.0 / dataRate);
			imupre.gt.SecOfWeek = imupre.gt.SecOfWeek + 1.0 / dataRate;
			for (int i = 0; i < 3; i++)
			{
				imupre.accl[i] = imupre.accl[i] * (1 - 1.0 / (double)num) + imu.accl[i] * (1.0 / (double)num);
				imupre.gyro[i] = imupre.gyro[i] * (1 - 1.0 / (double)num) + imu.gyro[i] * (1.0 / (double)num);
			}
			imupre.dtheta = imupre.gyro;
			imupre.dvel = imupre.accl;
			mapImu[imupre.gt.SecOfWeek] = imupre;
		}
		mapImu[imu.gt.SecOfWeek] = imu;
		imupre = imu;
	}
	fclose(fp_ins);

}

/// <summary>
/// IMU 数据输出
/// </summary>
/// <param name="out"></param>
/// <param name="imu"></param>
/// <returns></returns>
ostream& operator<<(ostream& out, _IMU& imu) 
{
	out << imu.gt.Week << ' ' << fixed << setprecision(3) << imu.gt.SecOfWeek << setprecision(10)
		<< ' ' << imu.accl[0] << ' ' << imu.accl[1] << ' ' << imu.accl[2]
		<< ' ' << imu.gyro[0] << ' ' << imu.gyro[1] << ' ' << imu.gyro[2] << '\n';
	return out;
}

std::ofstream& operator<<(std::ofstream& out, _IMU& imu)
{
	out << imu.gt.Week << ' ' << fixed << setprecision(3) << imu.gt.SecOfWeek << setprecision(10)
		<< ' ' << imu.accl[0] << ' ' << imu.accl[1] << ' ' << imu.accl[2]
		<< ' ' << imu.gyro[0] << ' ' << imu.gyro[1] << ' ' << imu.gyro[2] << '\n';
	return out;
}

std::ofstream& operator<<(std::ofstream& out, _IMUError& imuError)
{
	out << fixed << setprecision(10) << imuError.ba(0) << ' ' << imuError.ba(1) << ' ' << imuError.ba(2) << ' '
		<< imuError.bg(0) << ' ' << imuError.bg(1) << ' ' << imuError.bg(2) << ' '
		<< imuError.Sa(0) << ' ' << imuError.Sa(1) << ' ' << imuError.Sa(2) << ' '
		<< imuError.Sg(0) << ' ' << imuError.Sg(1) << ' ' << imuError.Sg(2) << '\n';
	return out;
}

/// <summary>
/// INU输出补偿
/// </summary>
/// <param name="imu">IMU输出</param>
/// <param name="imuerror">IMU误差</param>
void imuCompensate(_IMU& imu, _IMUError& imuerror)
{
	//补偿零偏
	imu.dtheta = imu.dtheta - imuerror.bg * imu.dt;
	imu.dvel = imu.dvel - imuerror.ba * imu.dt;
	//补偿比例因子
	Vector Sg(3), Sa(3), ones(3, 1.0);
	Sg = ones + imuerror.Sg;
	Sa = ones + imuerror.Sa;
	imu.dtheta = imu.dtheta / Sg;
	imu.dvel = imu.dvel / Sa;
}

/// <summary>
/// 中间时刻IMU数据内插
/// </summary>
/// <param name="imu1"></param>
/// <param name="imu2"></param>
/// <param name="updatetime"></param>
/// <param name="imumid"></param>
void imuInterpolate(_IMU& imu1, _IMU& imu2, double updatetime, _IMU& imumid)
{
	if (imu1.gt.SecOfWeek > updatetime || imu2.gt.SecOfWeek < updatetime) {
		return;
	}

	double lamda = (updatetime - imu1.gt.SecOfWeek) / (imu2.gt.SecOfWeek - imu1.gt.SecOfWeek);

	imumid.gt.SecOfWeek = updatetime;
	imumid.dtheta = imu2.dtheta * lamda;
	imumid.dvel = imu2.dvel * lamda;
	imumid.dt = updatetime - imu1.gt.SecOfWeek;

	imu2.dtheta = imu2.dtheta - imumid.dtheta;
	imu2.dvel = imu2.dvel - imumid.dvel;
	imu2.dt = imu2.gt.SecOfWeek - imumid.gt.SecOfWeek;

}