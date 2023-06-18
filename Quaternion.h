#pragma once
#ifndef Quaternion_H

#define Quaternion_H

#include<iostream>
#include<stdio.h>
#include"Matrix.h"
#include"Vector.h"

class Quaternion
{
private:
	double q[4];
public:
	inline Quaternion()
	{
		q[0] = q[1] = q[2] = q[3] = 0.0;
	}
	inline Quaternion(double a, double b, double c, double d)
	{
		q[0] = a; q[1] = b; q[2] = c; q[3] = d;
	}
	inline Quaternion(const Quaternion& q)
	{
		this->q[0] = q.q[0];
		this->q[1] = q.q[1];
		this->q[2] = q.q[2];
		this->q[3] = q.q[3];
	}
	//运算符重载
	// 四元数相等
	inline Quaternion& operator=(const Quaternion& q)
	{
		this->q[0] = q.q[0];
		this->q[1] = q.q[1];
		this->q[2] = q.q[2];
		this->q[3] = q.q[3];
		return *this;
	}

	//四元数数乘
	inline Quaternion operator*(double a)
	{
		Quaternion M;
		M.q[0] = this->q[0] * a;
		M.q[1] = this->q[1] * a;
		M.q[2] = this->q[2] * a;
		M.q[3] = this->q[3] * a;
		return M;
	}
	//四元数乘法
	inline Quaternion operator*(Quaternion& q)
	{
		//Quaternion M;
		double a = this->q[0] * q.q[0] - this->q[1] * q.q[1] - this->q[2] * q.q[2] - this->q[3] * q.q[3];
		double b = this->q[0] * q.q[1] + this->q[1] * q.q[0] + this->q[2] * q.q[3] - this->q[3] * q.q[2];
		double c = this->q[0] * q.q[2] + this->q[2] * q.q[0] + this->q[3] * q.q[1] - this->q[1] * q.q[3];
		double d = this->q[0] * q.q[3] + this->q[3] * q.q[0] + this->q[1] * q.q[2] - this->q[2] * q.q[1];
		Quaternion M(a, b, c, d);
		return M;
	}
	//四元数加减
	inline Quaternion operator+(Quaternion& q)
	{
		Quaternion M;
		M.q[0] = this->q[0] + q.q[0];
		M.q[1] = this->q[1] + q.q[1];
		M.q[2] = this->q[2] + q.q[2];
		M.q[3] = this->q[3] + q.q[3];
		return M;
	}
	inline Quaternion operator-(Quaternion& q)
	{
		Quaternion M;
		M.q[0] = this->q[0] - q.q[0];
		M.q[1] = this->q[1] - q.q[1];
		M.q[2] = this->q[2] - q.q[2];
		M.q[3] = this->q[3] - q.q[3];
		return M;
	}
	//四元数除以个数
	inline Quaternion operator/(double a)
	{
		Quaternion M;
		M.q[0] = this->q[0] / a;
		M.q[1] = this->q[1] / a;
		M.q[2] = this->q[2] / a;
		M.q[3] = this->q[3] / a;
		return M;
	}
	//共轭
	inline Quaternion operator~()
	{
		Quaternion M;
		M.q[0] = this->q[0];
		M.q[1] = -this->q[1];
		M.q[2] = -this->q[2];
		M.q[3] = -this->q[3];
		return M;
	}
	//模长
	inline double Norm()
	{
		double sum = 0.0;
		for (int i = 0; i < 4; i++)sum += this->q[i] * this->q[i];
		return sqrt(sum);
	}
	//求逆
	inline Quaternion Inverse()
	{
		Quaternion M = ~(*this);
		Quaternion N;
		N = M / (pow((*this).Norm(), 2));
		return N;
	}
	//归一化
	inline Quaternion Normalization()
	{
		Quaternion M = *this / (this->Norm());
		return M;
	}
	//获取某个值
	inline double operator()(int num)
	{
		return this->q[num];
	}


};

#endif // !Quaternion_H
