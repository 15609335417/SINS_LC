#pragma once
#ifndef LC_H
#include<stdio.h>
#include<fstream>
#include"Time.h"
#include"INS.h"
#include"Attitude.h"
#include"Matrix.h"
#include"Vector.h"
#include"GNSS.h"

#define TIME_ALIGN_ERR 0.001
#define tau 3600.0
#define deltag 9.8
#define sizeOfZuptWindow 50
#define IsLoseLock 0
#define LoseLockTime 97070.000
#define LoseLockTimeInterval 120
#define t_start 357697.000
struct Res
{
	GPST gt;
	double pos[3];
	double vel[3];
	double att[3];
	double ba[3];
	double bg[3];
	double Sa[3];
	double Sg[3];
	double deltax[21];//状态向量
	EulerAngles EA;
	Res()
	{
		for (int i = 0; i < 3; i++)
			pos[i] = vel[i] = att[i]= ba[i]=bg[i]=Sa[i]=Sg[i] = 0.0;
		for (int i = 0; i < 21; i++)
			deltax[i] = 0.0;
	}
};
//参考数据文件读取
void ReadResData(char* FileName, Res* data);

//判断是否需要更新
int isToUpdate(double imutimepre, double imutimecur, double updatetime);

//KF_Predict
void _SetF(const _IMUPVA& pvapre, const _IMU& imucur, Matrix& F);
void _SetG(const _IMUPVA& pvapre, Matrix& G);
void _Setq(Matrix&q,  double VRW, double ARW, double Sigma_bg, double sigma_ba, double Sigma_Sg, double Sigma_Sa, double interval);

//KF_Update
void _SetH(const _IMUPVA& pvacur, const _IMU& imucur, Vector& lb, Matrix& H);
void _SetR(Matrix& R, _GNSS& data);
void _SetZ(Vector& Z, _IMUPVA& pvacur, _GNSS& data, Vector& Lb );
void _SetP(Matrix& P, double* std_pos, double* std_vel, double* std_att, double Sigma_bg, double Sigma_ba, double Sigma_Sg, double Sigma_Sa);
void _SetP(Matrix& P);

//Kalman 滤波
void KF_Predict(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imu1, _IMU& imu2, _IMUError& imuerror, Matrix& qk, Matrix& pk_1, Vector xk);
void KF_Update(_GNSS& gnssdata, _IMUPVA& pvacur, _IMU& imucur, Vector& lb, Matrix& pk_1, Vector& xk);
void KF_Update_NHC(_IMUPVA& pvacur, _IMU& imucur, Vector& lb, Matrix& pk_1, Vector& xk);
void KF_Update_ZUPT(_IMUPVA& pvacur, _IMU& imucur, Matrix& pk_1, Vector& xk);

//闭环反馈
void stateFeedback(_IMUPVA& pva, _IMU& imu, Vector& xk, _IMUError& imuerror);

//零速修正
bool isZeroSpeed(std::map<double,_IMU>::iterator mapImu,const std::map<double,_IMU>::iterator&mapEnd);




//状态传播
//void insPropagation(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imu1, _IMU& imu2, _IMUError& imuerror, Matrix& qk, Matrix& pk_1,Vector xk);
//bool isZUPT(_IMU* imucur);

//GNSS更新
//void gnssUpdate(_GNSS& gnssdata, _IMUPVA&pvacur,  _IMU& imucur, Vector& lb,  Matrix& pk_1, Vector& xk);
//void gnssUpdate_ZUPT(PVA& pvacur, IMU& imucur, Matrix& pk_1, Vector& xk);
//void gnssUpdate_NHC(PVA& pvacur, IMU& imucur, Matrix& pk_1, Vector& xk, Vector& L_odo);
//void gnssUpdate_ODO_NHC(Odometer& odo, PVA& pvacur, IMU& imucur, Matrix& pk_1, Vector& xk, Vector& L_ODO);
//void gnssUpdate_ODO(Odometer& odo, PVA& pvacur, IMU& imucur, Matrix& pk_1, Vector& xk, Vector& L_ODO);
//
//void KF_GINS_Update(_GNSS& gnssdata, _IMUPVA& pvacur, _IMU& imucur, Vector& lb, Matrix& pk_1, Vector& xk, bool IsZUPT);
//void KF_Update(Odometer& odo, PVA& pvacur, IMU& imucur, IMUError& imuerror, Matrix& pk_1, Vector& xk, Vector& L_ODO, bool ZUPTstatus, bool NHCstatus, bool ODOstatus, bool ODO_NHCstatus, bool Headingstatus);


//结果输出
//void resultOut(_IMUPVA& pvacur, Res& ResData);
//void resultFileOut(_IMUPVA& pvacur, std::ofstream& resultfile);
//void resulterrorOut(_IMUPVA& pvacur, Res& ResData, std::ofstream& resultfile);


#endif // !LC_H
