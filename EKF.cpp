#include"EKF.h"
/// <summary>
/// 参考文件读取
/// </summary>
/// <param name="FileName"></param>
/// <param name="data"></param>
void ReadResData(char* FileName, Res* data)
{
	FILE* fp_res;
	char buffer[256];
	fopen_s(&fp_res, FileName, "r+b");
	int i = 0;
	if (fp_res == 0)
	{
		printf("Open Error!");
		return;
	}
	while (feof(fp_res) == 0)
	{
		fgets(buffer, 256, fp_res);
		sscanf_s(buffer, "%hd%lf%lf%lf%lf%lf%lf%lf%lf%lf%lf",
			&data[i].gt.Week, &data[i].gt.SecOfWeek, &data[i].pos[0], &data[i].pos[1], &data[i].pos[2],
			&data[i].vel[0], &data[i].vel[1], &data[i].vel[2],
			&data[i].att[0], &data[i].att[1], &data[i].att[2]);
		i++;
	}
	fclose(fp_res);
}

/// <summary>
/// F矩阵
/// </summary>
/// <param name="pvapre">上一时刻的位置速度姿态</param>
/// <param name="imucur">本时刻的IMU输出</param>
/// <param name="F"></param>
void _SetF(const _IMUPVA& pvapre, const _IMU& imucur, Matrix& F)
{
	double _F[441] = { 0 };
	double Frr[9] = { 0 };
	double RM = _SetRM(pvapre.pos(0));
	double RN = _SetRN(pvapre.pos(0));
	double gp = _Setgp(pvapre.pos(0), pvapre.pos(2));
	//位置误差
	//11
	Frr[0] = -pvapre.vel(2) / (RM + pvapre.pos(2)); _F[0] = Frr[0];
	Frr[2] = pvapre.vel(0) / (RM + pvapre.pos(2)); _F[2] = Frr[2];
	Frr[3] = pvapre.vel(1) * tan(pvapre.pos(0)) / (RN + pvapre.pos(2)); _F[21] = Frr[3];
	Frr[4] = -(pvapre.vel(2) + pvapre.vel(0) * tan(pvapre.pos(0))) / (RN + pvapre.pos(2)); _F[22] = Frr[4];
	Frr[5] = pvapre.vel(1) / (RN + pvapre.pos(2)); _F[23] = Frr[5];
	//速度误差
	//21
	double Fvr[9] = { 0 };
	Fvr[0] = -2 * pvapre.vel(1) * omega_e * cos(pvapre.pos(0)) / (RM + pvapre.pos(2)) - pvapre.vel(1) * pvapre.vel(1) / (cos(pvapre.pos(0)) * cos(pvapre.pos(0)) * (RM + pvapre.pos(2)) * (RN + pvapre.pos(2)));
	Fvr[2] = pvapre.vel(0) * pvapre.vel(2) / ((RM + pvapre.pos(2)) * (RM + pvapre.pos(2))) - pvapre.vel(1) * pvapre.vel(1) * tan(pvapre.pos(0)) / (pow((RN + pvapre.pos(2)), 2));
	Fvr[3] = 2 * omega_e * (pvapre.vel(0) * cos(pvapre.pos(0)) - pvapre.vel(2) * sin(pvapre.pos(0))) / (RM + pvapre.pos(2)) + pvapre.vel(0) * pvapre.vel(1) / (pow(cos(pvapre.pos(0)), 2) * (RM + pvapre.pos(2)) * (RN + pvapre.pos(2)));
	Fvr[5] = (pvapre.vel(1) * pvapre.vel(2) + pvapre.vel(0) * pvapre.vel(1) * tan(pvapre.pos(0))) / (pow((RN + pvapre.pos(2)), 2));
	Fvr[6] = 2 * omega_e * pvapre.vel(1) * sin(pvapre.pos(0)) / (RM + pvapre.pos(2));
	Fvr[8] = -pow(pvapre.vel(1), 2) / (pow((RN + pvapre.pos(2)), 2)) - pow(pvapre.vel(0), 2) / (pow((RM + pvapre.pos(2)), 2)) + 2 * gp / (sqrt(RM * RN) + pvapre.pos(2));
	_F[63] = Fvr[0]; _F[65] = Fvr[2]; _F[84] = Fvr[3]; _F[86] = Fvr[5]; _F[105] = Fvr[6]; _F[107] = Fvr[8];
	//22
	double Fvv[9] = { 0 };
	Fvv[0] = pvapre.vel(2) / (RM + pvapre.pos(2)); _F[66] = Fvv[0];
	Fvv[1] = -2 * (omega_e * sin(pvapre.pos(0)) + pvapre.vel(1) * tan(pvapre.pos(0)) / (RN + pvapre.pos(2))); _F[67] = Fvv[1];
	Fvv[2] = pvapre.vel(0) / (RM + pvapre.pos(2)); _F[68] = Fvv[2];
	Fvv[3] = 2 * omega_e * sin(pvapre.pos(0)) + pvapre.vel(1) * tan(pvapre.pos(0)) / (RN + pvapre.pos(2)); _F[87] = Fvv[3];
	Fvv[4] = (pvapre.vel(2) + pvapre.vel(0) * tan(pvapre.pos(0))) / (RN + pvapre.pos(2)); _F[88] = Fvv[4];
	Fvv[5] = 2 * omega_e * cos(pvapre.pos(0)) + pvapre.vel(1) / (RN + pvapre.pos(2)); _F[89] = Fvv[5];
	Fvv[6] = -2 * pvapre.vel(0) / (RM + pvapre.pos(2)); _F[108] = Fvv[6];
	Fvv[7] = -2 * (omega_e * cos(pvapre.pos(0)) + pvapre.vel(1) / (RN + pvapre.pos(2))); _F[109] = Fvv[7];
	//姿态误差
	//31
	double Fpr[9] = { 0 };
	Fpr[0] = -omega_e * sin(pvapre.pos(0)) / (RM + pvapre.pos(2)); _F[126] = Fpr[0];
	Fpr[2] = pvapre.vel(1) / (pow((RN + pvapre.pos(2)), 2)); _F[128] = Fpr[2];
	Fpr[5] = -pvapre.vel(0) / (pow((RM + pvapre.pos(2)), 2)); _F[149] = Fpr[5];
	Fpr[6] = -omega_e * cos(pvapre.pos(0)) / (RM + pvapre.pos(2)) - pvapre.vel(1) / (pow(cos(pvapre.pos(0)), 2) * (RM + pvapre.pos(2)) * (RN + pvapre.pos(2)));
	_F[168] = Fpr[6];
	Fpr[8] = -pvapre.vel(1) * tan(pvapre.pos(0)) / (pow((RN + pvapre.pos(2)), 2)); _F[170] = Fpr[8];
	//32
	double Fpv[9] = { 0 };
	Fpv[1] = 1 / (RN + pvapre.pos(2)); _F[130] = Fpv[1];
	Fpv[3] = -1 / (RM + pvapre.pos(2)); _F[150] = Fpv[3];
	Fpv[7] = -tan(pvapre.pos(0)) / (RN + pvapre.pos(2)); _F[172] = Fpv[7];
	//12
	_F[3] = 1; _F[25] = 1; _F[47] = 1;
	//23
	Matrix Cn_b(3, 3); Cn_b = pvapre.cbn.CbR;
	Vector acc(3); acc = (Vector)imucur.dvel / imucur.dt;
	Vector R_23(3); R_23 = Cn_b * acc;
	Matrix R23(3, 3); R23 = Antisymmetry(R_23);
	_F[69] = R23(0, 0); _F[70] = R23(0, 1); _F[71] = R23(0, 2);
	_F[90] = R23(1, 0); _F[91] = R23(1, 1); _F[92] = R23(1, 2);
	_F[111] = R23(2, 0); _F[112] = R23(2, 1); _F[113] = R23(2, 2);
	//Cnb
	//25
	_F[75] = Cn_b(0, 0); _F[76] = Cn_b(0, 1); _F[77] = Cn_b(0, 2);
	_F[96] = Cn_b(1, 0); _F[97] = Cn_b(1, 1); _F[98] = Cn_b(1, 2);
	_F[117] = Cn_b(2, 0); _F[118] = Cn_b(2, 1); _F[119] = Cn_b(2, 2);
	//27
	Matrix R27(3, 3);
	R27 = Cn_b * diag(acc);
	_F[81] = R27(0, 0); _F[82] = R27(0, 1); _F[83] = R27(0, 2);
	_F[102] = R27(1, 0); _F[103] = R27(1, 1); _F[104] = R27(1, 2);
	_F[123] = R27(2, 0); _F[124] = R27(2, 1); _F[125] = R27(2, 2);
	//33
	Matrix R33(3, 3);
	Vector omega_ien(3), omega_enn(3), omega_inn(3);
	omega_ien = _omega_ien(pvapre.pos(0));
	omega_enn = _omega_enn(pvapre.pos(0), pvapre.pos(2), pvapre.vel(0), pvapre.vel(1), pvapre.vel(2));
	omega_inn = omega_ien + omega_enn;
	R33 = Antisymmetry(omega_inn) * (-1);
	_F[132] = R33(0, 0); _F[133] = R33(0, 1); _F[134] = R33(0, 2);
	_F[153] = R33(1, 0); _F[154] = R33(1, 1); _F[155] = R33(1, 2);
	_F[174] = R33(2, 0); _F[175] = R33(2, 1); _F[176] = R33(2, 2);

	//34
	Matrix R34(3, 3);
	R34 = Cn_b * (-1);
	_F[135] = R34(0, 0); _F[136] = R34(0, 1); _F[137] = R34(0, 2);
	_F[156] = R34(1, 0); _F[157] = R34(1, 1); _F[158] = R34(1, 2);
	_F[177] = R34(2, 0); _F[178] = R34(2, 1); _F[179] = R34(2, 2);
	//36
	Matrix R36(3, 3);
	Vector omega(3); omega = (Vector)imucur.dtheta / imucur.dt;//
	R36 = R34 * diag(omega);
	_F[141] = R36(0, 0); _F[142] = R36(0, 1); _F[143] = R36(0, 2);
	_F[162] = R36(1, 0); _F[163] = R36(1, 1); _F[164] = R36(1, 2);
	_F[183] = R36(2, 0); _F[184] = R36(2, 1); _F[185] = R36(2, 2);
	//44

	Matrix Rxx(3, 3);
	Matrix I(3);
	Rxx = I * (-1 / tau);
	_F[198] = Rxx(0, 0);
	_F[220] = Rxx(1, 1);
	_F[242] = Rxx(2, 2);
	_F[264] = Rxx(0, 0);
	_F[286] = Rxx(1, 1);
	_F[308] = Rxx(2, 2);
	_F[330] = Rxx(0, 0);
	_F[352] = Rxx(1, 1);
	_F[374] = Rxx(2, 2);
	_F[396] = Rxx(0, 0);
	_F[418] = Rxx(1, 1);
	_F[440] = Rxx(2, 2);
	F = _F;
}

/// <summary>
/// G矩阵
/// </summary>
/// <param name="pvapre">上一时刻</param>
/// <param name="G"></param>
void _SetG(const _IMUPVA& pvapre, Matrix& G)
{
	double _G[378] = { 0 };
	Matrix Cn_b(3, 3); Cn_b = pvapre.cbn.CbR;
	_G[54] = Cn_b(0, 0); _G[55] = Cn_b(0, 1); _G[56] = Cn_b(0, 2);
	_G[72] = Cn_b(1, 0); _G[73] = Cn_b(1, 1); _G[74] = Cn_b(1, 2);
	_G[90] = Cn_b(2, 0); _G[91] = Cn_b(2, 1); _G[92] = Cn_b(2, 2);
	_G[111] = Cn_b(0, 0); _G[112] = Cn_b(0, 1); _G[113] = Cn_b(0, 2);
	_G[129] = Cn_b(1, 0); _G[130] = Cn_b(1, 1); _G[131] = Cn_b(1, 2);
	_G[147] = Cn_b(2, 0); _G[148] = Cn_b(2, 1); _G[149] = Cn_b(2, 2);
	for (int i = 0; i < 12; i++)
	{
		_G[168 + i * 19] = 1;
	}
	G = _G;
}

/// <summary>
/// q矩阵
/// </summary>
void _Setq(Matrix& q, double VRW, double ARW, double Sigma_bg, double sigma_ba, double Sigma_Sg, double Sigma_Sa, double interval)
{
	double _q[324] = { 0 };
	for (int i = 0; i < 3; i++)
	{
		_q[0 + i * 19] = pow(VRW, 2);
		_q[57 + i * 19] = pow(ARW, 2);
		_q[114 + i * 19] = pow(Sigma_bg, 2) * 2 / interval;
		_q[171 + i * 19] = pow(sigma_ba, 2) * 2 / interval;
		_q[228 + i * 19] = pow(Sigma_Sg, 2) * 2 / interval;
		_q[285 + i * 19] = pow(Sigma_Sa, 2) * 2 / interval;
	}
	q = _q;
}

/// <summary>
/// H 矩阵
/// </summary>
/// <param name="pvacur">本时刻</param>
/// <param name="imucur">本时刻</param>
/// <param name="lb">杆臂</param>
/// <param name="H"></param>
void _SetH(const _IMUPVA& pvacur, const _IMU& imucur, Vector& lb, Matrix& H)
{
	double _H[126] = { 0.0 };
	double RM = _SetRM(pvacur.pos(0));
	double RN = _SetRN(pvacur.pos(0));
	double gp = _Setgp(pvacur.pos(0), pvacur.pos(2));
	//11
	for (int i = 0; i < 3; i++)
	{
		_H[0 + i * 22] = 1;
	}
	//13
	Matrix Cn_b(3, 3); Cn_b = pvacur.cbn.CbR;
	Vector R_13(3); R_13 = Cn_b * lb;
	Matrix R13(3, 3); R13 = Antisymmetry(R_13);
	_H[6] = R13(0, 0); _H[7] = R13(0, 1); _H[8] = R13(0, 2);
	_H[27] = R13(1, 0); _H[28] = R13(1, 1); _H[29] = R13(1, 2);
	_H[48] = R13(2, 0); _H[49] = R13(2, 1); _H[50] = R13(2, 2);
	//22
	for (int i = 0; i < 3; i++)
	{
		_H[66 + i * 22] = 1;
	}

	//23
	Matrix R23(3, 3);
	Vector omega_ien(3); omega_ien = _omega_ien(pvacur.pos(0));
	Vector omega_enn(3); omega_enn = _omega_enn(pvacur.pos(0), pvacur.pos(2), pvacur.vel(0), pvacur.vel(1), pvacur.vel(2));//ω^n _en
	Vector omega_inn(3); omega_inn = omega_ien + omega_enn;
	Vector omega(3); omega = (Vector)imucur.dtheta / imucur.dt;
	Vector R_23(3); R_23 = Cn_b * (lb * omega);
	R23 = Antisymmetry(omega_inn) * R13 * (-1);
	R23 = R23 - Antisymmetry(R_23);
	_H[69] = R23(0, 0); _H[70] = R23(0, 1); _H[71] = R23(0, 2);
	_H[90] = R23(1, 0); _H[91] = R23(1, 1); _H[92] = R23(1, 2);
	_H[111] = R23(2, 0); _H[112] = R23(2, 1); _H[113] = R23(2, 2);
	//24
	Matrix R24(3, 3);
	R24 = Cn_b * Antisymmetry(lb) * (-1);//
	_H[72] = R24(0, 0); _H[73] = R24(0, 1); _H[74] = R24(0, 2);
	_H[93] = R24(1, 0); _H[94] = R24(1, 1); _H[95] = R24(1, 2);
	_H[114] = R24(2, 0); _H[115] = R24(2, 1); _H[116] = R24(2, 2);
	//26
	Matrix R26(3, 3);
	R26 = Cn_b * Antisymmetry(lb) * diag(omega) * (-1);
	_H[78] = R26(0, 0); _H[79] = R26(0, 1); _H[80] = R26(0, 2);
	_H[99] = R26(1, 0); _H[100] = R26(1, 1); _H[101] = R26(1, 2);
	_H[120] = R26(2, 0); _H[121] = R26(2, 1); _H[122] = R26(2, 2);


	H = _H;
}

/// <summary>
/// P 协方差矩阵
/// </summary>
void _SetP(Matrix& P, double* std_pos, double* std_vel, double* std_att, double Sigma_bg, double Sigma_ba, double Sigma_Sg, double Sigma_Sa)
{
	double p[441] = { 0.0 };
	for (int i = 0; i < 3; i++)
	{
		p[i * 22] = pow(std_pos[i], 2);
		p[66 + i * 22] = pow(std_vel[i], 2);
		p[132 + i * 22] = pow(std_att[i], 2);
		p[198 + i * 22] = pow(Sigma_bg, 2);
		p[264 + i * 22] = pow(Sigma_ba, 2);
		p[330 + i * 22] = pow(Sigma_Sg, 2);
		p[396 + i * 22] = pow(Sigma_Sa, 2);
	}
	P = p;
}

void _SetP(Matrix& P)
{
	double p[441] = { 0.0 };
	//ZUPT和Res最优
	for (int i = 0; i < 21; i++)
	{
		p[i * 22] = 1e-6;

	}
	P = p;
}


/// <summary>
/// R 矩阵
/// </summary>
void _SetR(Matrix& R, _GNSS& data)
{
	double arrayRk[36] = { 0 };
	for (int i = 0; i < 3; i++)
	{
		arrayRk[i * 7] = pow(data.stdpos[i], 2);
		arrayRk[21 + i * 7] = pow(data.stdvel[i], 2);
	}
	R = arrayRk;
}

/// <summary>
/// Z 矩阵
/// </summary>
void _SetZ(Vector& Z, _IMUPVA& pvacur, _GNSS& data, Vector& Lb)
{
	Matrix NED2BLH(3, 3), BLH2NED(3, 3), cbn(3, 3);
	NED2BLH = _NED2BLH(_SetRM(pvacur.pos(0)), _SetRN(pvacur.pos(0)), pvacur.pos(0), pvacur.pos(2));
	BLH2NED = _BLH2NED(_SetRM(pvacur.pos(0)), _SetRN(pvacur.pos(0)), pvacur.pos(0), pvacur.pos(2));
	cbn = pvacur.cbn.CbR;
	Vector antenna_pos(3), dz_pos(3), gnssdata(3);
	antenna_pos = pvacur.pos + NED2BLH * cbn * Lb;
	//antenna_pos = pvacur.pos - NED2BLH * cbn * Lb;               //jieguo zhengque lun 
	gnssdata = data.pos;
	dz_pos = BLH2NED * (antenna_pos - gnssdata);
	double arrayZ[6] = { 0.0 };
	for (int i = 0; i < 3; i++)
	{
		arrayZ[i] = dz_pos(i);
		arrayZ[3 + i] = pvacur.vel(i) - data.vel[i];
	}
	Z = arrayZ;
}

/// <summary>
/// 判断此时刻是否需要更新
/// </summary>
/// <returns>0：不需要；1：GNSS在前后两个IMU时刻之间，需要内插；3：GNSS刚好在当前IMU时刻，直接更新即可</returns>
int isToUpdate(double imutimepre, double imutimecur, double updatetime)
{
	if (fabs(imutimecur - updatetime) < EPS && fabs(imutimepre - updatetime) - 1.0 / dataRate < EPS)
		return 3;
	else if (imutimepre < updatetime && updatetime < imutimecur) {
		// 更新时间在imutime1和imutime2之间,//// 但不靠近任何一个
		// updatetime is between imutime1 and imutime2, but not near to either
		return 1;
	}
	else
		return 0;
}

//int isToUpdate(double imutime, double updatetime)
//{
//	if (fabs(imutime - updatetime) < EPS)
//		return 3;
//	//else if (imutime - updatetime < EPS && fabs(imutime - updatetime) - 1.0 / dataRate < EPS)
//	//	return 1;
//	else
//		return 0;
//}

///// <summary>
///// 机械编排，状态传播
///// </summary>
///// <param name="pvapre">上一时刻的pva</param>
///// <param name="pvacur">本时刻</param>
///// <param name="imu1">上一时刻的imu输出</param>
///// <param name="imu2">本时刻</param>
///// <param name="imuerror">imu误差</param>
///// <param name="phi">状态传播矩阵</param>
///// <param name="qk">系统噪声</param>
///// <param name="pk_1">xk_1的协方差矩阵</param>
//void insPropagation(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imu1, _IMU& imu2, _IMUError& imuerror, Matrix& qk, Matrix& pk_1, Vector xk)
//{
//
//	// 对当前IMU数据(imucur)补偿误差, 上一IMU数据(imupre)已经补偿过了
//	imuCompensate(imu2,imuerror);
//	//判断本时刻是否为起始时刻
//	if (fabs(imu2.gt.SecOfWeek - t_start) < EPS)
//	{
//		pvacur = pvapre;
//		return;
//	}
//	// IMU状态更新(机械编排算法)
//	Mechanican(pvapre, pvacur, imu1, imu2);
//	
//	//系统噪声传播
//	Matrix F(21, 21);
//	Matrix phi(21, 21);
//	Matrix I(21, 1.0);
//	Matrix Gk(21, 18);
//	Matrix Q(21, 21);
//	Matrix Pk_k1(21, 21);
//	_SetF(pvapre, imu2, F);
//	phi = I + F * imu2.dt;
//	_SetG(pvapre, Gk);
//	Q = (phi * Gk * qk * Gk.Transpose() * phi.Transpose() + Gk * qk * Gk.Transpose()) * 0.5 * imu2.dt;
//	Pk_k1 = phi * pk_1 * phi.Transpose() + Q;
//	pk_1 = Pk_k1;
//	xk = phi * xk;
//}



/// <summary>
/// 是否进行零速更新
/// </summary>
/// <param name="imucur"></param>
/// <returns></returns>
//bool isZUPT(_IMU* imucur)
//{
//	double sum = 0.0;
//	for (int i = 0; i < ZUPTsize; i++)
//	{
//		sum += fabs((imucur + i)->dvel.Norm() - deltag / 200.0);
//	}
//	if (sum < 0.0665)
//		return true;
//	else
//		return false;
//}
//
///// <summary>
///// GNSS 更新
///// </summary>
//void gnssUpdate(_GNSS& gnssdata, _IMUPVA& pvacur, _IMU& imucur, Vector& lb, Matrix& pk_1, Vector& xk)
//{
//	//判断本时刻是否为起始时刻
//	if (fabs(imucur.gt.SecOfWeek - t_start) < EPS)
//	{
//		return;
//	}
//	//kalman滤波
//			//量测更新
//			//HZR
//			//H
//	Matrix H(6, 21);
//	Matrix R(6, 6);
//	Vector Z(6);
//	Matrix K(21, 6);
//	Matrix Pk(21, 21);
//	Matrix BLH2NED(3, 3);
//	Matrix I(21, 1.0);
//	_SetH(pvacur, imucur, lb, H);
//	_SetR(R, gnssdata);
//	_SetZ(Z, pvacur, gnssdata,lb);
//	//量测更新
//	//pk_1.Show();
//	K = pk_1 * H.Transpose() * (H * pk_1 * H.Transpose() + R).Inverse();
//	xk = xk + K * (Z - H * xk);
//	Pk = (I - K * H) * pk_1 * (I - K * H).Transpose() + K * R * K.Transpose();
//	pk_1 = Pk;
//
//}

/// <summary>
/// GNSS + ZUPT
/// </summary>
//void gnssUpdate_ZUPT(PVA& pvacur, IMU& imucur, Matrix& pk_1, Vector& xk)
//{
//	Matrix H(3, 21);
//	Matrix R(3, 3);
//	Vector Z(3);
//	Matrix K(21, 3);
//	Matrix I(21, 1.0);
//	/// H
//	double _H[63] = { 0.0 };
//	//22
//	for (int i = 0; i < 3; i++)
//	{
//		_H[3 + i * 22] = 1;
//	}
//	H = _H;
//	/// R
//	double arrayRk[9] = { 0 };
//	arrayRk[0] = arrayRk[4] = arrayRk[8] = 1e-2;//需要调整
//	R = arrayRk;
//	/// Z
//	Z = pvacur.vel;
//	//量测更新
//	K = pk_1 * H.Transpose() * (H * pk_1 * H.Transpose() + R).Inverse();
//	xk = xk + K * (Z - H * xk);
//	pk_1 = (I - K * H) * pk_1 * (I - K * H).Transpose() + K * R * K.Transpose();
//
//}

/// <summary>
/// GNSS + ODO
/// </summary>
//void gnssUpdate_ODO(Odometer& odo, PVA& pvacur, IMU& imucur, Matrix& pk_1, Vector& xk, Vector& L_ODO)
//{
//	Matrix H(1, 21);
//	Matrix R(1, 1);
//	Vector Z(1);
//	Matrix K(21, 1);
//	Matrix Pk(21, 21);
//	Matrix I(21, 1.0);
//
//	Matrix cbr(3, 1.0);
//	Matrix cbn(3, 3); cbn = pvacur.cbn.CbR;
//	Matrix cnb(3, 3); cnb = cbn.Transpose();
//
//	Vector omega_ibb(3); omega_ibb = imucur.dtheta / imucur.dt;
//	Vector omega_nbb(3); omega_nbb = omega_ibb - cnb * (_omega_ien(pvacur.pos(0)) + _omega_enn(pvacur.pos(0), pvacur.pos(2), pvacur.vel(0), pvacur.vel(1), pvacur.vel(2)));
//
//	///H
//	double _H[21] = { 0.0 };
//	////12
//	double arrayh[3] = { 1,0,0 }; Matrix _h(1, 3); _h = arrayh;
//	Matrix R12(1, 3); R12 = _h * cbr * cnb;
//	_H[3] = R12(0, 0); _H[4] = R12(0, 1); _H[5] = R12(0, 2);
//	//测试
//	//13
//	Matrix R13(1, 3); R13 = _h * cbr * cnb * Antisymmetry(pvacur.vel) * (-1);
//	_H[6] = R13(0, 0); _H[7] = R13(0, 1); _H[8] = R13(0, 2);
//	//14
//	Matrix R14(1, 3); R14 = _h * cbr * Antisymmetry(L_ODO) * (-1);
//	_H[9] = R14(0, 0); _H[10] = R14(0, 1); _H[11] = R14(0, 2);
//	//16
//	Matrix R16(1, 3); R16 = _h * cbr * Antisymmetry(L_ODO) * diag(omega_ibb) * (-1);
//	_H[15] = R16(0, 0); _H[16] = R16(0, 1); _H[17] = R16(0, 2);
//
//
//
//	H = _H;
//	///R
//	double arrayRk[2] = { 0 };
//	arrayRk[0] = 1e-2;
//	R = arrayRk;
//	///Z
//	//INS->速度-odo测量速度
//	double arrayv_wheel[3] = { odo.vel };
//	Vector v_wheel(1); v_wheel = arrayv_wheel;
//	Vector hat_v_wheel(1);
//	hat_v_wheel = _h * cbr * (cnb * pvacur.vel + omega_nbb * L_ODO);
//	Z = hat_v_wheel - v_wheel;
//	//量测更新
//	K = pk_1 * H.Transpose() * (H * pk_1 * H.Transpose() + R).Inverse();
//	xk = xk + K * (Z - H * xk);
//	///xk.Show();
//	pk_1 = (I - K * H) * pk_1 * (I - K * H).Transpose() + K * R * K.Transpose();
//}

/// <summary>
/// GNSS + NHC
/// </summary>
//void gnssUpdate_NHC(PVA& pvacur, IMU& imucur, Matrix& pk_1, Vector& xk,Vector &L_odo)
//{
//	//H
//	Matrix H(2, 21);
//	Matrix R(2, 2);
//	Vector Z(2);
//	Matrix K(21, 2);
//	Matrix Pk(21, 21);
//	Matrix I(21, 1.0);
//
//	Matrix cbr(3, 1.0);
//	Matrix cbn(3, 3); cbn = pvacur.cbn.CbR;
//	Matrix cnb(3, 3); cnb = cbn.Inverse();
//
//	Vector omega_ibb(3); omega_ibb = imucur.dtheta / imucur.dt;
//	Vector omega_nbb(3); omega_nbb = omega_ibb - cnb * (_omega_ien(pvacur.pos(0)) + _omega_enn(pvacur.pos(0), pvacur.pos(2), pvacur.vel(0), pvacur.vel(1), pvacur.vel(2)));
//
//	///H
//	double _H[42] = { 0.0 };
//	//12
//	double arrayh[6] = { 0,1,0,0,0,1 };
//	Matrix h(2, 3); h = arrayh;
//	Matrix R12(2, 3); R12 = h * cbr * cnb;
//	_H[3] = R12(0, 0); _H[4] = R12(0, 1); _H[5] = R12(0, 2);
//	_H[24] = R12(1, 0); _H[25] = R12(1, 1); _H[26] = R12(1, 2);
//	//13
//	Matrix R13(2, 3); R13 = h * cbr * cnb * Antisymmetry(pvacur.vel) * (-1);
//	_H[6] = R13(0, 0); _H[7] = R13(0, 1); _H[8] = R13(0, 2);
//	_H[27] = R13(1, 0); _H[28] = R13(1, 1); _H[29] = R13(1, 2);
//	//14
//	Matrix R14(2, 3); R14 = h * cbr * Antisymmetry(L_odo) * (-1);
//	_H[9] = R14(0, 0); _H[10] = R14(0, 1); _H[11] = R14(0, 2);
//	_H[30] = R14(1, 0); _H[31] = R14(1, 1); _H[32] = R14(1, 2);
//	//16
//	Matrix R16(2, 3); R16 = h * cbr * Antisymmetry(L_odo) * diag(omega_ibb) * (-1);
//	_H[15] = R16(0, 0); _H[16] = R16(0, 1); _H[17] = R16(0, 2);
//	_H[36] = R16(1, 0); _H[37] = R16(1, 1); _H[38] = R16(1, 2);
//
//	H = _H;
//	///R
//	double arrayRk[4] = { 0 };
//	arrayRk[0] = arrayRk[3] = 1;
//	R = arrayRk;
//	///Z
//	Z = h * cbr * (cnb * pvacur.vel + omega_nbb * L_odo);
//	//量测更新
//	K = pk_1 * H.Transpose() * (H * pk_1 * H.Transpose() + R).Inverse();
//	xk = xk + K * (Z - H * xk);
//	pk_1 = (I - K * H) * pk_1 * (I - K * H).Transpose() + K * R * K.Transpose();
//}
//
///// <summary>
///// ODO+NHC
///// </summary>
//void gnssUpdate_ODO_NHC(Odometer& odo, PVA& pvacur, IMU& imucur, Matrix& pk_1, Vector& xk, Vector& L_ODO)
//{
//	Matrix H(3, 21);
//	Matrix R(3, 3);
//	Vector Z(3);
//	Matrix K(21, 3);
//	Matrix I(21, 1.0);
//
//	///H
//	double _H[63] = { 0.0 };
//	//12
//	Matrix cbr(3, 1.0);
//	Matrix cbn(3, 3); cbn = pvacur.cbn.CbR;
//	Matrix cnb(3, 3); cnb = cbn.Inverse();
//	Matrix crn(3, 3); crn = cbr.Inverse() * cbn;
//	Matrix R12(3, 3); R12 = cbr * cnb;
//	_H[3] = R12(0, 0); _H[4] = R12(0, 1); _H[5] = R12(0, 2);
//	_H[24] = R12(1, 0); _H[25] = R12(1, 1); _H[26] = R12(1, 2);
//	_H[45] = R12(2, 0); _H[46] = R12(2, 1); _H[47] = R12(2, 2);
//	//13
//	Matrix R13(3, 3); R13 = cbr * cnb*Antisymmetry(pvacur.vel)*(-1);
//	_H[6] = R13(0, 0); _H[7] = R13(0, 1); _H[8] = R13(0, 2);
//	_H[27] = R13(1, 0); _H[28] = R13(1, 1); _H[29] = R13(1, 2);
//	_H[48] = R13(2, 0); _H[49] = R13(2, 1); _H[50] = R13(2, 2);
//	//14
//	Matrix R14(3, 3); R14 = cbr * Antisymmetry(L_ODO) * (-1);
//	_H[9] = R14(0, 0); _H[10] = R14(0, 1); _H[11] = R14(0, 2);
//	_H[30] = R14(1, 0); _H[31] = R14(1, 1); _H[32] = R14(1, 2);
//	_H[51] = R14(2, 0); _H[52] = R14(2, 1); _H[53] = R14(2, 2);
//	//16
//	Vector omega_ibb(3); omega_ibb = imucur.dtheta / imucur.dt;
//	Matrix R16(3, 3); R16 = cbr * Antisymmetry(L_ODO)*diag(omega_ibb) * (-1);
//	_H[15] = R16(0, 0); _H[16] = R16(0, 1); _H[17] = R16(0, 2);
//	_H[36] = R16(1, 0); _H[37] = R16(1, 1); _H[38] = R16(1, 2);
//	_H[57] = R16(2, 0); _H[58] = R16(2, 1); _H[59] = R16(2, 2);
//
//	H = _H;
//	///R
//	double arrayRk[9] = { 0 };
//	arrayRk[0] = arrayRk[4]=arrayRk[8] = 1e-0;
//	R = arrayRk;
//	///Z
//	//INS->速度-odo测量速度
//	double arrayv_wheel[3] = {odo.vel,0,0 };
//	Vector v_wheel(3); v_wheel = arrayv_wheel;
//	Vector omega_nbb(3); omega_nbb = omega_ibb - cnb * (_omega_ien(pvacur.pos(0)) + _omega_enn(pvacur.pos(0), pvacur.pos(2), pvacur.vel(0), pvacur.vel(1), pvacur.vel(2)));
//	Vector hat_v_wheel(3);
//	hat_v_wheel = cbr * cnb * pvacur.vel + cbr * Antisymmetry(omega_nbb) * L_ODO;
//	Z = hat_v_wheel - v_wheel;
//	//量测更新
//	K = pk_1 * H.Transpose() * (H * pk_1 * H.Transpose() + R).Inverse();
//	xk = xk + K * (Z - H * xk);
//	pk_1 = (I - K * H) * pk_1 * (I - K * H).Transpose() + K * R * K.Transpose();
//}


/// <summary>
/// GNSS 参与
/// </summary>
/// <param name="ZUPTstatus">是否进行ZUPT</param>
/// <param name="NHCstatus">是否进行NHC</param>
/// <param name="ODOstatus">是否进行ODO</param>
/// <param name="Headingstatus">是否进行航向角更新</param>
//void KF_GINS_Update(_GNSS& gnssdata, _IMUPVA& pvacur, _IMU& imucur, Vector& lb, Matrix& pk_1, Vector& xk, bool IsZUPT)
//{
//	//GNSS 是否失锁
//	if (IsLoseLock && gnssdata.gt.SecOfWeek - LoseLockTime > EPS && gnssdata.gt.SecOfWeek - (LoseLockTime + LoseLockTimeInterval) < EPS)
//			return;
//	else
//		gnssUpdate(gnssdata, pvacur, imucur, lb, pk_1, xk);
//}

/// <summary>
/// GNSS 不参与
/// </summary>
//void KF_Update(Odometer& odo, PVA& pvacur, IMU& imucur, IMUError& imuerror, Matrix& pk_1, Vector& xk, Vector& L_ODO, bool ZUPTstatus, bool NHCstatus, bool ODOstatus, bool ODO_NHCstatus, bool Headingstatus)
//{
//	if (ZUPTstatus)
//	{
//		gnssUpdate_ZUPT(pvacur, imucur, pk_1, xk);
//		stateFeedback(pvacur, imucur, xk, imuerror);
//	}
//	else if (NHCstatus)
//	{
//		gnssUpdate_NHC(pvacur, imucur, pk_1, xk, L_ODO);
//		stateFeedback(pvacur, imucur, xk, imuerror);
//	}
//	else if (ODOstatus)
//	{
//		if (fabs(imucur.time - odo.time) < EPS)
//		{
//			gnssUpdate_ODO(odo, pvacur, imucur, pk_1, xk, L_ODO);
//			stateFeedback(pvacur, imucur, xk, imuerror);
//		}
//		else return;
//	}
//	else if (ODO_NHCstatus)
//	{
//		if (fabs(imucur.time - odo.time) < EPS)
//		{
//			gnssUpdate_ODO_NHC(odo, pvacur, imucur, pk_1, xk, L_ODO);
//			stateFeedback(pvacur, imucur, xk, imuerror);
//		}
//		else return;
//	}
//	else return;
//}

/// <summary>
/// 闭环反馈
/// </summary>
/// <param name="pva"></param>
/// <param name="xk"></param>
/// <param name="imuerror"></param>
void stateFeedback(_IMUPVA& pva, _IMU& imu, Vector& xk, _IMUError& imuerror)
{
	////判断本时刻是否为起始时刻
	//if (fabs(pva.time - t_start) < EPS)
	//{
	//	return;
	//}
	Vector d_pos(3);
	Vector d_vel(3);
	Vector d_att(3);
	Matrix I(3, 1.0);
	Matrix NED2BLH(3, 3);
	Matrix cbn(3, 3); cbn = pva.cbn.CbR;
	double zero[21] = { 0.0 };
	Quaternion qpn;
	double RM = _SetRM(pva.pos(0));
	double RN = _SetRN(pva.pos(0));
	NED2BLH = _NED2BLH(RM, RN, pva.pos(0), pva.pos(2));
	d_pos = NED2BLH * xk(0, 2);
	d_vel = xk(3, 5);
	d_att = xk(6, 8);
	pva.pos = pva.pos - d_pos;
	pva.vel = pva.vel - d_vel;
	Vector2Quaternion(d_att, qpn);
	pva.qbn = qpn * pva.qbn;
	Quaternion2EulerAngles(pva.qbn, pva.att);
	Quaternion2DCM(pva.qbn, pva.cbn);

	//cbn = (I - Antisymmetry(d_att)).Inverse() * cbn;
	//DCMatrix2DCM(cbn, pva.cbn);
	//DCMatrix2EulerAngles(cbn, pva.att);
	//DCMatrix2Quaternion(cbn, pva.qbn);

	imuerror.bg = imuerror.bg + xk(9, 11);
	imuerror.ba = imuerror.ba + xk(12, 14);
	imuerror.Sg = imuerror.Sg + xk(15, 17);
	imuerror.Sa = imuerror.Sa + xk(18, 20);

	xk = zero;

}

/// <summary>
/// 是否为零速
/// </summary>
bool isZeroSpeed(std::map<double, _IMU>::iterator mapImu, const std::map<double, _IMU>::iterator& mapEnd)
{
	int i = 0;
	double sum = 0.0;
	for (; i < sizeOfZuptWindow; i++)
	{
		if (mapImu == mapEnd)break;
		sum += fabs(mapImu->second.dvel.Norm() - deltag / dataRate);
		mapImu++;
	}
	if (sum/i < 0.0002)
		return true;
	else
		return false;

}


///// <summary>
///// 结果控制台输出
///// </summary>
///// <param name="pvacur"></param>
///// <param name="ResData"></param>
//void resultOut(_IMUPVA& pvacur, Res& ResData)
//{
//	Res Result;
//	Result.gt.SecOfWeek = pvacur.time;
//	for (int m = 0; m < 3; m++)
//	{
//		Result.pos[m] = pvacur.pos(m);
//		Result.vel[m] = pvacur.vel(m);
//	}
//	Result.pos[0] = rad2deg(pvacur.pos(0));
//	Result.pos[1] = rad2deg(pvacur.pos(1));
//	Result.att[0] = pvacur.att.phi; Result.att[1] = pvacur.att.theta; Result.att[2] = pvacur.att.psi;
//	//printf("估值：%9.3lf %14.10lf %14.10lf %8.3lf %8.3lf %8.3lf %8.3lf %13.8lf %13.8lf %13.8lf\n", Result.gt.SecOfWeek, Result.pos[0], Result.pos[1], Result.pos[2], Result.vel[0], Result.vel[1], Result.vel[2], Result.att[0], Result.att[1], Result.att[2]);
//	//printf("参考：%9.3lf %14.10lf %14.10lf %8.3lf %8.3lf %8.3lf %8.3lf %13.8lf %13.8lf %13.8lf\n", ResData.gt.SecOfWeek, ResData.pos[0], ResData.pos[1], ResData.pos[2], ResData.vel[0], ResData.vel[1], ResData.vel[2], ResData.att[0], ResData.att[1], ResData.att[2]);
//	//printf("误差：%9.3lf %14.10lf %14.10lf %8.3lf %8.3lf %8.3lf %8.3lf %13.8lf %13.8lf %13.8lf\n", ResData.gt.SecOfWeek, Result.pos[0] - ResData.pos[0], Result.pos[1] - ResData.pos[1], Result.pos[2] - ResData.pos[2], Result.vel[0] - ResData.vel[0], Result.vel[1] - ResData.vel[1], Result.vel[2] - ResData.vel[2], Result.att[0] - ResData.att[0], Result.att[1] - ResData.att[1], Result.att[2] - ResData.att[2]);
//	//printf("%.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf %.12lf\n", Result.gt.SecOfWeek, Result.pos[0], Result.pos[1], Result.pos[2], Result.vel[0], Result.vel[1], Result.vel[2], Result.att[0], Result.att[1], Result.att[2]);
//
//}
//
///// <summary>
///// 结果文件输出
///// </summary>
///// <param name="pvacur"></param>
///// <param name="resultfile"></param>
//void resultFileOut(_IMUPVA& pvacur, std::ofstream& resultfile)
//{
//	resultfile << std::fixed;
//	resultfile << std::setw(9)<< std::setprecision(3)<< pvacur.time << ' ';
//	resultfile << std::setw(14) << std::setprecision(10) << rad2deg(pvacur.pos(0)) << ' ' << rad2deg(pvacur.pos(1)) << ' ';
//	resultfile << std::setw(8) << std::setprecision(3) << pvacur.pos(2) << ' ';
//	resultfile << std::setw(8) << std::setprecision(3) << pvacur.vel(0) << ' '<< pvacur.vel(1) << ' '<< pvacur.vel(2) << ' ';
//	resultfile << std::setw(13) << std::setprecision(8) << pvacur.att.phi << ' ' << pvacur.att.theta << ' ' << pvacur.att.psi ;
//	resultfile << '\n';
//}
//
///// <summary>
///// 结果与参考结果误差
///// </summary>
///// <param name="pvacur"></param>
///// <param name="ResData"></param>
///// <param name="resultfile"></param>
//void resulterrorOut(_IMUPVA& pvacur, Res& ResData, std::ofstream& resultfile)
//{
//	Res Result;
//	Result.gt.SecOfWeek = pvacur.time;
//	for (int m = 0; m < 3; m++)
//	{
//		Result.vel[m] = pvacur.vel(m) - ResData.vel[m];
//	}
//	Result.pos[0] = rad2deg(pvacur.pos(0)) - ResData.pos[0];
//	Result.pos[1] = rad2deg(pvacur.pos(1)) - ResData.pos[1];
//	Result.pos[2] = pvacur.pos(2) - ResData.pos[2];
//	Result.att[0] = pvacur.att.phi - ResData.att[0];
//	Result.att[1] = pvacur.att.theta - ResData.att[1];
//	Result.att[2] = pvacur.att.psi - ResData.att[2];
//	resultfile << std::fixed;
//	resultfile << std::setw(9) << std::setprecision(3) << Result.gt.SecOfWeek << ' ';
//	resultfile << std::setw(14) << std::setprecision(10) << Result.pos[0] << ' ' << Result.pos[1] << ' ';
//	resultfile << std::setw(8) << std::setprecision(3) << Result.pos[2] << ' ';
//	resultfile << std::setw(8) << std::setprecision(3) << Result.vel[0] << ' ' << Result.vel[1] << ' ' << Result.vel[2] << ' ';
//	resultfile << std::setw(13) << std::setprecision(8) << Result.att[0] << ' ' << Result.att[1] << ' ' << Result.att[2];
//	resultfile << '\n';
//
//}




/// <summary>
/// kalman 一步预测
/// </summary>
void KF_Predict(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imu1, _IMU& imu2, _IMUError& imuerror, Matrix& qk, Matrix& pk_1, Vector xk)
{
	// 对当前IMU数据(imucur)补偿误差, 上一IMU数据(imupre)已经补偿过了
	imuCompensate(imu2, imuerror);
	//判断本时刻是否为起始时刻
	//if (fabs(imu2.gt.SecOfWeek - t_start) < EPS)
	//{
	//	pvacur = pvapre;
	//	return;
	//}
	// IMU状态更新(机械编排算法)
	Mechanican(pvapre, pvacur, imu1, imu2);

	//系统噪声传播
	Matrix F(21, 21);
	Matrix phi(21, 21);
	Matrix I(21, 1.0);
	Matrix Gk(21, 18);
	Matrix Q(21, 21);
	Matrix Pk_k1(21, 21);
	_SetF(pvapre, imu2, F);
	phi = I + F * imu2.dt;
	_SetG(pvapre, Gk);
	Q = (phi * Gk * qk * Gk.Transpose() * phi.Transpose() + Gk * qk * Gk.Transpose()) * 0.5 * imu2.dt;
	Pk_k1 = phi * pk_1 * phi.Transpose() + Q;
	pk_1 = Pk_k1;
	xk = phi * xk; //实为xk_1

}

/// <summary>
/// kalman 滤波量测更新
/// </summary>
void KF_Update(_GNSS& gnssdata, _IMUPVA& pvacur, _IMU& imucur, Vector& lb, Matrix& pk_1, Vector& xk)
{
	////判断本时刻是否为起始时刻
	//if (fabs(imucur.gt.SecOfWeek - t_start) < EPS)
	//{
	//	return;
	//}

	//量测更新
	Matrix H(6, 21);
	Matrix R(6, 6);
	Vector Z(6);
	Matrix K(21, 6);
	Matrix Pk(21, 21);
	Matrix BLH2NED(3, 3);
	Matrix I(21, 1.0);
	_SetH(pvacur, imucur, lb, H);
	_SetR(R, gnssdata);
	_SetZ(Z, pvacur, gnssdata, lb);
	K = pk_1 * H.Transpose() * (H * pk_1 * H.Transpose() + R).Inverse();
	xk = xk + K * (Z - H * xk);
	Pk = (I - K * H) * pk_1 * (I - K * H).Transpose() + K * R * K.Transpose();
	pk_1 = Pk;


}

/// <summary>
/// NHC 约束
/// </summary>
void KF_Update_NHC(_IMUPVA& pvacur, _IMU& imucur, Vector& lb, Matrix& pk_1, Vector& xk)
{
	//H
	Matrix H(2, 21);
	Matrix R(2, 2);
	Vector Z(2);
	Matrix K(21, 2);
	Matrix Pk(21, 21);
	Matrix I(21, 1.0);

	Matrix cbr(3, 1.0);
	Matrix cbn(3, 3); cbn = pvacur.cbn.CbR;
	Matrix cnb(3, 3); cnb = cbn.Inverse();

	Vector omega_ibb(3); omega_ibb = imucur.dtheta / imucur.dt;
	Vector omega_nbb(3); omega_nbb = omega_ibb - cnb * (_omega_ien(pvacur.pos(0)) + _omega_enn(pvacur.pos(0), pvacur.pos(2), pvacur.vel(0), pvacur.vel(1), pvacur.vel(2)));

	///H
	double _H[42] = { 0.0 };
	//12
	double arrayh[6] = { 0,1,0,0,0,1 };
	Matrix h(2, 3); h = arrayh;
	Matrix R12(2, 3); R12 = h * cbr * cnb;
	_H[3] = R12(0, 0); _H[4] = R12(0, 1); _H[5] = R12(0, 2);
	_H[24] = R12(1, 0); _H[25] = R12(1, 1); _H[26] = R12(1, 2);
	//13
	Matrix R13(2, 3); R13 = h * cbr * cnb * Antisymmetry(pvacur.vel) * (-1);
	_H[6] = R13(0, 0); _H[7] = R13(0, 1); _H[8] = R13(0, 2);
	_H[27] = R13(1, 0); _H[28] = R13(1, 1); _H[29] = R13(1, 2);
	//14
	Matrix R14(2, 3); R14 = h * cbr * Antisymmetry(lb) * (-1);
	_H[9] = R14(0, 0); _H[10] = R14(0, 1); _H[11] = R14(0, 2);
	_H[30] = R14(1, 0); _H[31] = R14(1, 1); _H[32] = R14(1, 2);
	//16
	Matrix R16(2, 3); R16 = h * cbr * Antisymmetry(lb) * diag(omega_ibb) * (-1);
	_H[15] = R16(0, 0); _H[16] = R16(0, 1); _H[17] = R16(0, 2);
	_H[36] = R16(1, 0); _H[37] = R16(1, 1); _H[38] = R16(1, 2);

	H = _H;
	///R
	double arrayRk[4] = { 0 };
	arrayRk[0] = arrayRk[3] = 1;
	R = arrayRk;
	///Z
	Z = h * cbr * (cnb * pvacur.vel + omega_nbb * lb);
	//量测更新
	K = pk_1 * H.Transpose() * (H * pk_1 * H.Transpose() + R).Inverse();
	xk = xk + K * (Z - H * xk);
	pk_1 = (I - K * H) * pk_1 * (I - K * H).Transpose() + K * R * K.Transpose();

}

/// <summary>
/// ZUPT
/// </summary>
void KF_Update_ZUPT(_IMUPVA& pvacur, _IMU& imucur, Matrix& pk_1, Vector& xk)
{
	Matrix H(3, 21);
	Matrix R(3, 3);
	Vector Z(3);
	Matrix K(21, 3);
	Matrix I(21, 1.0);
	/// H
	double _H[63] = { 0.0 };
	//22
	for (int i = 0; i < 3; i++)
	{
		_H[3 + i * 22] = 1;
	}
	H = _H;
	/// R
	double arrayRk[9] = { 0 };
	arrayRk[0] = arrayRk[4] = arrayRk[8] = 1e-4;//需要调整
	R = arrayRk;
	/// Z
	Z = pvacur.vel;
	//量测更新
	K = pk_1 * H.Transpose() * (H * pk_1 * H.Transpose() + R).Inverse();
	xk = xk + K * (Z - H * xk);
	pk_1 = (I - K * H) * pk_1 * (I - K * H).Transpose() + K * R * K.Transpose();

}
