#pragma once
#ifndef Attitude_H

#define Attitude_H


#include<iostream>
#include<stdio.h>
#include<cmath>
#include<iomanip>
#include"Matrix.h"
#include"Vector.h"
#include"Quaternion.h"
#include"Coor.h"


struct EulerAngles
{
	double phi;//横滚角，右机翼下压为正-180~180
	double theta;//俯仰角，抬头为正；-90~90
	double psi;//航向角，北偏东为正；0~360;-180~180
	EulerAngles()
	{
		phi = theta = psi = 0.0;
	}
	EulerAngles(double Roll, double Pitch, double Heading)
	{
		phi = Roll;
		theta = Pitch;
		psi = Heading;
	}
};
struct DCM
{
	double CbR[9];//须先定义姿态角
	DCM()
	{
		for (int i = 0; i < 9; i++)
			CbR[i] = 0.0;
	}
};
struct RotationVector
{
	double phi[3];
	Vector RV;
	RotationVector()
	{
		for (int i = 0; i < 3; i++)
			phi[i] = 0.0;
		RV = phi;
	}
};


/// <summary>
/// b->n
/// </summary>
/// <param name="El"></param>
/// <param name="Cn_b"></param>
inline void AttitudeMatrix(EulerAngles& EA, Matrix& Cn_b)
{
	EulerAngles El;
	El.phi = deg2rad(EA.phi); El.theta = deg2rad(EA.theta); El.psi = deg2rad(EA.psi);
	double arrayCn_b[9] = { cos(El.theta) * cos(El.psi),-cos(El.phi) * sin(El.psi) + sin(El.phi) * sin(El.theta) * cos(El.psi),sin(El.phi) * sin(El.psi) + cos(El.phi) * sin(El.theta) * cos(El.psi),
			cos(El.theta) * sin(El.psi),cos(El.phi) * cos(El.psi) + sin(El.phi) * sin(El.theta) * sin(El.psi),-sin(El.phi) * cos(El.psi) + cos(El.phi) * sin(El.theta) * sin(El.psi),
			-sin(El.theta),sin(El.phi) * cos(El.theta),cos(El.phi) * cos(El.theta) };
	Cn_b = arrayCn_b;
}
//姿态角表示方法之间的转换
///欧拉角转方向余弦矩阵
inline void EulerAngles2DCM(EulerAngles& EA, DCM& dcm)
{
	dcm.CbR[0] = cos(deg2rad(EA.theta)) * cos(deg2rad(EA.psi));
	dcm.CbR[1] = -cos(deg2rad(EA.phi)) * sin(deg2rad(EA.psi)) + sin(deg2rad(EA.phi)) * sin(deg2rad(EA.theta)) * cos(deg2rad(EA.psi));
	dcm.CbR[2] = sin(deg2rad(EA.phi)) * sin(deg2rad(EA.psi)) + cos(deg2rad(EA.phi)) * sin(deg2rad(EA.theta)) * cos(deg2rad(EA.psi));
	dcm.CbR[3] = cos(deg2rad(EA.theta)) * sin(deg2rad(EA.psi));
	dcm.CbR[4] = cos(deg2rad(EA.phi)) * cos(deg2rad(EA.psi)) + sin(deg2rad(EA.phi)) * sin(deg2rad(EA.theta)) * sin(deg2rad(EA.psi));
	dcm.CbR[5] = -sin(deg2rad(EA.phi)) * cos(deg2rad(EA.psi)) + cos(deg2rad(EA.phi)) * sin(deg2rad(EA.theta)) * sin(deg2rad(EA.psi));
	dcm.CbR[6] = -sin(deg2rad(EA.theta));
	dcm.CbR[7] = sin(deg2rad(EA.phi)) * cos(deg2rad(EA.theta));
	dcm.CbR[8] = cos(deg2rad(EA.phi)) * cos(deg2rad(EA.theta));
}
///欧拉角转四元数
inline void EulerAngles2Quaternion(EulerAngles& EA, Quaternion& q)
{
	double q0 = cos(deg2rad(EA.phi / 2.0)) * cos(deg2rad(EA.theta / 2.0)) * cos(deg2rad(EA.psi / 2.0)) + sin(deg2rad(EA.phi / 2.0)) * sin(deg2rad(EA.theta / 2.0)) * sin(deg2rad(EA.psi / 2.0));
	double q1 = sin(deg2rad(EA.phi / 2.0)) * cos(deg2rad(EA.theta / 2.0)) * cos(deg2rad(EA.psi / 2.0)) - cos(deg2rad(EA.phi / 2.0)) * sin(deg2rad(EA.theta / 2.0)) * sin(deg2rad(EA.psi / 2.0));
	double q2 = cos(deg2rad(EA.phi / 2.0)) * sin(deg2rad(EA.theta / 2.0)) * cos(deg2rad(EA.psi / 2.0)) + sin(deg2rad(EA.phi / 2.0)) * cos(deg2rad(EA.theta / 2.0)) * sin(deg2rad(EA.psi / 2.0));
	double q3 = cos(deg2rad(EA.phi / 2.0)) * cos(deg2rad(EA.theta / 2.0)) * sin(deg2rad(EA.psi / 2.0)) - sin(deg2rad(EA.phi / 2.0)) * sin(deg2rad(EA.theta / 2.0)) * cos(deg2rad(EA.psi / 2.0));
	Quaternion p(q0, q1, q2, q3);
	q = p;
}
///方向余弦转欧拉角
inline void DCM2EulerAngles(DCM& dcm, EulerAngles& EA)
{
	EA.theta = rad2deg(atan(-dcm.CbR[6] / sqrt(pow(dcm.CbR[7], 2) + pow(dcm.CbR[8], 2))));
	double a = 0.0;
	if (fabs(dcm.CbR[6]) < 0.999)
	{
		EA.phi = rad2deg(atan2(dcm.CbR[7], dcm.CbR[8]));
		EA.psi = rad2deg(atan2(dcm.CbR[3], dcm.CbR[0]));
	}
	else
	{
		if (dcm.CbR[6] <= -0.999)
			a = atan2(dcm.CbR[5] - dcm.CbR[1], dcm.CbR[2] + dcm.CbR[4]);
		else if (dcm.CbR[6] >= 0.999)
			a = atan2(dcm.CbR[5] + dcm.CbR[1], dcm.CbR[2] - dcm.CbR[4]) + PI;

	}
	if (EA.psi < EPS)
		EA.psi += 360;
}
inline void DCMatrix2EulerAngles(Matrix& dcm, EulerAngles& att)
{
	att.theta = atan(-dcm(2, 0) / sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));
	att.theta = rad2deg(att.theta);
	//dcm(2,0)<0.999
	att.phi = atan2(dcm(2, 1), dcm(2, 2));
	att.phi = rad2deg(att.phi);
	att.psi = atan2(dcm(1, 0), dcm(0, 0));
	att.psi = rad2deg(att.psi);
	//航向角0-360；
	if (att.psi < EPS)
		att.psi += 360;
}
inline void DCMatrix2DCM(Matrix& M, DCM& cbn)
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cbn.CbR[i * 3 + j] = M(i, j);
		}
	}
}
///方向余弦转四元数
inline double Max(double* P, int Num)
{
	double max = 0;
	for (int i = 0; i < Num; i++)
	{
		if (P[i] > max)
			max = P[i];
	}
	return max;
}
inline void DCM2Quaternion(DCM& dcm, Quaternion& q)
{
	double q1 = 0.0, q2 = 0.0, q3 = 0.0, q4 = 0.0;
	Matrix CBR(3, 3); CBR = dcm.CbR;
	double P[4] = { 1 + CBR.tr(),1 + 2 * CBR(0,0) - CBR.tr(),1 + 2 * CBR(1,1) - CBR.tr(),1 + 2 * CBR(2,2) - CBR.tr() };
	double _Max = Max(P, 4);
	if (_Max == P[0])
	{
		q1 = 0.5 * sqrt(P[0]); q2 = (CBR(2, 1) - CBR(1, 2)) / (4 * q1);
		q3 = (CBR(0, 2) - CBR(2, 0)) / (4 * q1);
		q4 = (CBR(1, 0) - CBR(0, 1)) / (4 * q1);
	}
	if (_Max == P[1])
	{
		q2 = 0.5 * sqrt(P[1]); q3 = (CBR(1, 0) + CBR(0, 1)) / (4 * q2);
		q4 = (CBR(0, 2) + CBR(2, 0)) / (4 * q2);
		q2 = (CBR(2, 1) - CBR(1, 2)) / (4 * q1);
	}
	if (_Max == P[2])
	{
		q3 = 0.5 * sqrt(P[2]); q4 = (CBR(2, 1) + CBR(1, 2)) / (4 * q3);
		q1 = (CBR(0, 2) - CBR(2, 0)) / (4 * q3);
		q2 = (CBR(1, 0) + CBR(0, 1)) / (4 * q3);
	}
	if (_Max == P[3])
	{
		q4 = 0.5 * sqrt(P[3]); q1 = (CBR(1, 0) - CBR(0, 1)) / (4 * q4);
		q2 = (CBR(0, 2) + CBR(2, 0)) / (4 * q4);
		q3 = (CBR(2, 1) + CBR(1, 2)) / (4 * q4);
	}
	Quaternion p(q1, q2, q3, q4);
	q = p;
	if (q(0) < 0)
		q = q * (-1.0);
}
inline void DCMatrix2Quaternion(Matrix& CBR, Quaternion& q)
{
	double q1 = 0.0, q2 = 0.0, q3 = 0.0, q4 = 0.0;
	double P[4] = { 1 + CBR.tr(),1 + 2 * CBR(0,0) - CBR.tr(),1 + 2 * CBR(1,1) - CBR.tr(),1 + 2 * CBR(2,2) - CBR.tr() };
	double _Max = Max(P, 4);
	if (_Max == P[0])
	{
		q1 = 0.5 * sqrt(P[0]); q2 = (CBR(2, 1) - CBR(1, 2)) / (4 * q1);
		q3 = (CBR(0, 2) - CBR(2, 0)) / (4 * q1);
		q4 = (CBR(1, 0) - CBR(0, 1)) / (4 * q1);
	}
	if (_Max == P[1])
	{
		q2 = 0.5 * sqrt(P[1]); q3 = (CBR(1, 0) + CBR(0, 1)) / (4 * q2);
		q4 = (CBR(0, 2) + CBR(2, 0)) / (4 * q2);
		q2 = (CBR(2, 1) - CBR(1, 2)) / (4 * q1);
	}
	if (_Max == P[2])
	{
		q3 = 0.5 * sqrt(P[2]); q4 = (CBR(2, 1) + CBR(1, 2)) / (4 * q3);
		q1 = (CBR(0, 2) - CBR(2, 0)) / (4 * q3);
		q2 = (CBR(1, 0) + CBR(0, 1)) / (4 * q3);
	}
	if (_Max == P[3])
	{
		q4 = 0.5 * sqrt(P[3]); q1 = (CBR(1, 0) - CBR(0, 1)) / (4 * q4);
		q2 = (CBR(0, 2) + CBR(2, 0)) / (4 * q4);
		q3 = (CBR(2, 1) + CBR(1, 2)) / (4 * q4);
	}
	Quaternion p(q1, q2, q3, q4);
	q = p;
	if (q(0) < 0)
		q = q * (-1.0);
}
///四元数转方向余弦矩阵
inline void Quaternion2DCM(Quaternion& q, DCM& dcm)
{
	dcm.CbR[0] = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
	dcm.CbR[1] = 2 * (q(1) * q(2) - q(0) * q(3));
	dcm.CbR[2] = 2 * (q(1) * q(3) + q(0) * q(2));
	dcm.CbR[3] = 2 * (q(1) * q(2) + q(0) * q(3));
	dcm.CbR[4] = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
	dcm.CbR[5] = 2 * (q(2) * q(3) - q(0) * q(1));
	dcm.CbR[6] = 2 * (q(1) * q(3) - q(0) * q(2));
	dcm.CbR[7] = 2 * (q(2) * q(3) + q(0) * q(1));
	dcm.CbR[8] = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);

}
///四元数转等效旋转矢量
///设q^R _b 对应的等效旋转矢量为phi^R _b
inline void Quaternion2RotationVector(Quaternion& q, RotationVector& r)
{
	double f = acos(q(0));
	double b = 0.0;
	if (fabs(f) > EPS)
		b = 2 * f / sin(f);
	else
		b = PI;
	r.phi[0] = q(1) * b;
	r.phi[1] = q(2) * b;
	r.phi[2] = q(3) * b;
}
///四元数转矢量
inline void Quaternion2Vector(Quaternion& q, Vector& r)
{
	RotationVector rv;
	Quaternion2RotationVector(q, rv);
	r = rv.phi;
}
///四元数转欧拉角
inline void Quaternion2EulerAngles(Quaternion& q, EulerAngles& El)
{
	DCM dcm;
	Quaternion2DCM(q, dcm);
	DCM2EulerAngles(dcm, El);
}
///等效旋转矢量转方向余弦矩阵
inline Matrix Antisymmetry(Vector& m)
{
	//只演示三维的反对称矩阵
	double arrayq[9] = { 0,-m(2),m(1),m(2),0,-m(0),-m(1),m(0),0 };
	Matrix q(3, 3);
	q = arrayq;
	return q;
}
inline Matrix diag(Vector& m)
{
	double arrayq[9] = {m(0), 0,0,0,m(1),0,0,0,m(2)};
	Matrix q(3, 3);
	q = arrayq;
	return q;
}
inline void Vector2DCMatrix(Vector& phi, Matrix& dcm)
{
	Matrix I(3);
	double NormPhi = phi.Norm();
	dcm = I + Antisymmetry(phi) * (sin(NormPhi) / NormPhi) + Antisymmetry(phi) * Antisymmetry(phi) * ((1 - cos(NormPhi)) / (NormPhi * NormPhi));

}
inline void RotationVector2DCM(RotationVector& r, DCM& dcm)
{
	Matrix CbR(3, 3);
	Matrix I(3);
	Vector phi(3); phi = r.phi;
	double NormPhi = phi.Norm();
	CbR = I + Antisymmetry(phi) * (sin(NormPhi) / NormPhi) + Antisymmetry(phi) * Antisymmetry(phi) * ((1 - cos(NormPhi)) / (NormPhi * NormPhi));
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			dcm.CbR[i * 3 + j] = CbR(i, j);
		}
	}
}
///等效旋转矢量转四元数
inline void RotationVector2Quaternion(RotationVector& r, Quaternion& q)
{
	Vector phi(3); phi = r.phi;
	double q1 = 0.0;
	if (phi.Norm() < EPS)
	{
		q1 = 1;
		Quaternion Q(q1, 0, 0, 0);
		q = Q;
	}
	else
	{
		q1 = cos((phi * 0.5).Norm());
		Vector q_234(3);
		q_234 = phi * (sin((phi * 0.5).Norm()) / phi.Norm());
		Quaternion Q(q1, q_234(0), q_234(1), q_234(2));
		q = Q;
	}
}
///向量转四元数
inline void Vector2Quaternion(Vector& phi, Quaternion& q)
{
	double q1 = 0.0;
	if (phi.Norm() < EPS)
	{
		q1 = 1;
		Quaternion Q(q1, 0, 0, 0);
		q = Q;
	}
	else
	{
		q1 = cos((phi * 0.5).Norm());
		Vector q_234(3);
		q_234 = phi * (sin((phi * 0.5).Norm()) / phi.Norm());
		Quaternion Q(q1, q_234(0), q_234(1), q_234(2));
		q = Q;
	}
}

inline void _Setqne(Vector& pos,Quaternion& qne)
{
	double coslon, sinlon, coslat, sinlat;
	coslon = cos(pos(1) * 0.5);
	sinlon = sin(pos(1) * 0.5);
	coslat = cos(-PI * 0.25 - pos(0) * 0.5);
	sinlat = sin(-PI * 0.25 - pos(0) * 0.5);
	Quaternion quat(coslat * coslon, -sinlat * sinlon, sinlat * coslon, coslat * sinlon);
	qne = quat;
}
inline Vector _Setpos(Quaternion& qne, double height)
{
	double array[3]= { -2 * atan(qne(2) / qne(0)) - PI * 0.5, 2 * atan2(qne(3), qne(0)), height };
	Vector _pos(3); _pos = array;
	return _pos;
}
inline Matrix _SetDRi(Vector& pos)
{
	double RM = _a * (1 - e2) / pow(1 - e2 * pow(sin(pos(0)), 2), 1.5);
	double RN = _a / sqrt(1 - e2 * pow(sin(pos(0)), 2));
	double array[9] = { 1.0 / (RM + pos(2)),0.0,0.0,0.0,1.0 / ((RN + pos(2)) * cos(pos(0))),0.0,0.0,0.0,-1 };
	Matrix dri(3, 3); dri = array;
	return dri;
}
/// <summary>
/// BLH->NED
/// </summary>
/// <returns>BLH->NED</returns>
inline Matrix _BLH2NED(double RM, double RN, double b, double h)
{
	Matrix b2n(3, 3);
	double array_blh_ned[3] = { RM + h,(RN + h) * cos(b),-1 };
	Vector _blh_ned(3); _blh_ned = array_blh_ned;
	b2n = diag(_blh_ned);
	return b2n;
}
/// <summary>
/// NED->BLH
/// </summary>
/// <returns>BLH->NED</returns>
inline Matrix _NED2BLH(double RM, double RN, double b, double h)
{
	Matrix n2b(3, 3);
	double array_ned_blh[3] = { 1.0 / (RM + h),1.0 / ((RN + h) * cos(b)),-1 };
	Vector ned_blh(3); ned_blh = array_ned_blh;
	n2b = diag(ned_blh);
	return n2b;
}

inline std::ostream& operator<<(std::ostream& out, EulerAngles& att)
{
	out << std::fixed << std::setprecision(9) << att.phi << ' ' << att.theta << ' ' << att.psi << '\n';
	return out;
}
#endif // !Attitude_H
