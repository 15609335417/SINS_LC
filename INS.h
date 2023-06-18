#pragma once
#ifndef INS_H



#include"IMU.h"

struct _IMUPVA   //位置速度姿态
{
	double time;
	Vector pos;
	Vector vel;
	Vector velpre;
	EulerAngles att;
	DCM cbn;
	Quaternion qbn;
	_IMUPVA()
	{
		time = 0.0;
		double array[3] = { 0.0 };
		pos = array;
		vel = array;
		velpre = array;
	}
};
struct _INSResult
{
	GPST gt;
	_blh pos;//blh //弧度
	_ned vel;//ned
	double RM;
	double RN;
	double gp;
	EulerAngles EA;
	DCM dcm;
	_INSResult()
	{
		RM = 0.0;
		RN = 0.0;
		gp = 0.0;
	}
};
/*RM，RN*/
//phi 为纬度，单位弧度
inline double _SetRM(double phi)
{
	return _a * (1 - e2) / pow(1 - e2 * pow(sin(phi), 2), 1.5);
}
inline double _SetRN(double phi)
{
	return _a / sqrt(1 - e2 * pow(sin(phi), 2));
}
inline double _Setgp(double phi, double h)
{
	double g0 = 9.7803267715 * (1 + 0.0052790414 * pow(sin(phi), 2) + 0.0000232718 * pow(sin(phi), 4));
	double g = g0 - (3.087691089e-6 - 4.397731e-9 * pow(sin(phi), 2)) * h + 0.721e-12 * pow(h, 2);
	return g;
}
inline Vector _omega_ien(double phi)
{
	double arrayomega_ien[3] = { omega_e * cos(phi),0,-omega_e * sin(phi) };
	Vector omega_ien(3); omega_ien = arrayomega_ien;//ω^n _ie
	return omega_ien;
}
inline Vector _omega_enn(double phi, double h, double Vn, double Ve, double Vd)
{
	double arrayomega_enn[3] = { Ve / (_SetRN(phi) + h),-Vn / (_SetRM(phi) + h), -Ve * tan(phi) / (_SetRN(phi) + h) };
	Vector omega_enn(3); omega_enn = arrayomega_enn;//ω^n _en
	return omega_enn;
}
inline std::ostream& operator<<(std::ostream& out, _IMUPVA& pva)
{
	out << std::fixed << std::setprecision(3) << pva.time 
		<< " B:" << std::setprecision(10) << std::setw(15) << rad2deg(pva.pos(0)) 
		<< " L:" << std::setw(15) << rad2deg(pva.pos(1)) 
		<< " H:" << std::setprecision(4) << std::setw(13) << pva.pos(2) 
		<< " VN:"<< std::setw(9) << pva.vel(0) 
		<< " VE:" << std::setw(9) << pva.vel(1) 
		<< " VD:" << std::setw(9) << pva.vel(2)
		<< " Yaw:"<< std::setprecision(10) << std::setw(15) << pva.att.psi 
		<< " Pitch:" << std::setw(15) << pva.att.theta 
		<< " Roll:" << std::setw(15) << pva.att.phi << '\n';
	return out;
}
inline std::ofstream& operator<<(std::ofstream& out, _IMUPVA& pva)
{
	//BLH->ENU 
	XYZ base; base.x = -2267808.6227; base.y = 5009324.7699; base.z = 3221016.8928;
	BLH Base;
	XYZ2BLH(base, Base, _a, _alpha);
	BLH rover; rover.b = rad2deg(pva.pos(0)); rover.l = rad2deg(pva.pos(1)); rover.h = pva.pos(2);
	XYZ xRover;
	ENU Rover;
	BLH2XYZ(rover, xRover, _a, _alpha);
	XYZ2ENU(xRover, Base, Rover, _a, _alpha);
	//BLH2ENU(rover, Base, Rover, _a, _alpha);
	out << std::fixed << std::setprecision(3) << pva.time << ' '
		<< std::setprecision(12) << std::setw(18) << rad2deg(pva.pos(0)) << ' '
		<< std::setw(18) << rad2deg(pva.pos(1)) << ' '
		<< std::setprecision(4) << std::setw(13) << pva.pos(2) << ' '
		<< std::setw(9) << pva.vel(0) << ' '
		<< std::setw(9) << pva.vel(1) << ' '
		<< std::setw(9) << pva.vel(2) << ' '
		<< std::setprecision(12) << std::setw(18) << pva.att.psi << ' '
		<< std::setw(18) << pva.att.theta << ' '
		<< std::setw(18) << pva.att.phi << ' '
		<< std::setprecision(4) << std::setw(9) << Rover.e << ' '
		<< std::setw(9) << Rover.n << ' '
		<< std::setw(9) << Rover.u << ' '
		<< std::setprecision(4) << std::setw(9) << xRover.x << ' '
		<< std::setw(9) << xRover.y << ' '
		<< std::setw(9) << xRover.z << '\n';
	return out;
}

/*静态粗对准*/
EulerAngles initialAlignment(const _IMU& imu, const double* pos);
EulerAngles initialAlignment(const std::map<double, _IMU> mapImu, const double* pos);
/*速度更新*/
void velUpdate(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imupre, _IMU& imucur);
/*位置更新*/
void posUpdate(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imupre, _IMU& imucur);
/*姿态更新*/
void attUpdate(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imupre, _IMU& imucur);

void Mechanican(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imupre, _IMU& imucur);






#endif // !INS_H