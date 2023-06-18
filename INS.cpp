#include"INS.h"
/// <summary>
/// ����Ԫ��ʼ��׼
/// </summary>
/// <param name="imu">IMU����</param>
/// <returns>��̬���Ҿ���</returns>
EulerAngles initialAlignment(const _IMU& imu, const double* pos)
{
	double phi = deg2rad(pos[0]);
	double g = _Setgp(phi,pos[2]);
	double temp[3] = { 0.0,0.0,g };
	Vector gn(3); gn = temp;
	temp[0] = omega_e * cos(phi); temp[1] = 0.0; temp[2] = omega_e * sin(phi) * (-1);
	Vector omega_ien(3); omega_ien = temp;
	Vector omega_ibb(3); omega_ibb =(Vector)imu.dtheta * dataRate;
	Vector gb(3); gb = (Vector)imu.dvel * (-1.0) * dataRate;

	Vector vg(3); vg = gn / gn.Norm();
	Vector v_omega(3); v_omega = (gn * omega_ien) / (gn * omega_ien).Norm();
	Vector vg_omega(3); vg_omega = (gn * omega_ien * gn) / (gn * omega_ien * gn).Norm();
	Vector omega_g(3); omega_g = gb / gb.Norm();
	Vector omega_omega(3); omega_omega = (gb * omega_ibb) / (gb * omega_ibb).Norm();
	Vector omega_g_omega(3); omega_g_omega = (gb * omega_ibb * gb) / (gb * omega_ibb * gb).Norm();

	double temp1[9]= { vg(0),v_omega(0) ,vg_omega(0),
		vg(1),v_omega(1),vg_omega(1),
		vg(2),v_omega(2),vg_omega(2) };
	double temp2[9] = { omega_g(0),omega_g(1) ,omega_g(2),
		omega_omega(0),omega_omega(1),omega_omega(2),
		omega_g_omega(0),omega_g_omega(1),omega_g_omega(2) };

	Matrix mtemp1(3, 3); mtemp1 = temp1;
	Matrix mtemp2(3, 3); mtemp2 = temp2; 
	Matrix Cbn(3, 3); Cbn = mtemp1* mtemp2;
	EulerAngles att;
	DCM dcm;
	DCMatrix2DCM(Cbn, dcm);
	DCM2EulerAngles(dcm, att);
	return att;
}

EulerAngles initialAlignment(const std::map<double, _IMU> mapImu, const double* pos)
{
	//��ʼ��׼
	int noa = 0; ///numOfAlignment ��ʼ��׼��Ԫ��
	_IMU imuMean;
	EulerAngles initialAtt;
	for (auto it = mapImu.begin(); it != mapImu.end(); it++)
	{
		if (fabs(it->first - mapImu.begin()->first - toa) < EPS)
			break;//���ƣ��ڶ������ڶ�׼
		noa++;
		for (int i = 0; i < 3; i++)
		{
			imuMean.accl[i] += it->second.accl[i];
			imuMean.gyro[i] += it->second.gyro[i];
		}
	}
	for (int i = 0; i < 3; i++)
	{
		imuMean.accl[i] = imuMean.accl[i] / (double)noa;
		imuMean.gyro[i] = imuMean.gyro[i] / (double)noa;
	}
	imuMean.dvel = imuMean.accl; imuMean.dtheta = imuMean.gyro;
	initialAtt = initialAlignment(imuMean, pos);
	return initialAtt;

}

/// <summary>
/// �ٶȸ���
/// </summary>
void velUpdate(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imupre, _IMU& imucur)
{
	Vector d_vfb(3), d_vfn(3), d_vgn(3), gl(3), midvel(3), midpos(3);
	Vector temp1(3), temp2(3), temp3(3);
	Matrix cnn(3, 3), I33(3, 1.0), cbn(3, 3);
	Quaternion qne, qee, qnn, qbb, q1, q2;
	// ����������������Ȧ�뾶��î��Ȧ�뾶��������ת���ٶ�ͶӰ��nϵ, nϵ�����eϵת�����ٶ�ͶӰ��nϵ������ֵ
	double RM = _SetRM(pvapre.pos(0));
	double RN = _SetRN(pvapre.pos(0));
	Vector omega_ie_n(3), omega_en_n(3);
	omega_ie_n = _omega_ien(pvapre.pos(0));
	omega_en_n = _omega_enn(pvapre.pos(0), pvapre.pos(2), pvapre.vel(0), pvapre.vel(1), pvapre.vel(2));
	double gravity = _Setgp(pvapre.pos(0), pvapre.pos(2));
	//��תЧӦ��˫��������ЧӦ
	temp1 = imucur.dtheta * imucur.dvel / 2.0;
	temp2 = imupre.dtheta * imucur.dvel / 12.0;
	temp3 = imupre.dvel * imucur.dtheta / 12.0;
	//bϵ����������
	d_vfb = imucur.dvel + temp1 + temp2 + temp3;
	//����������ͶӰ��nϵ
	temp1 = (omega_ie_n + omega_en_n) * imucur.dt / 2.0;
	cnn = I33 - Antisymmetry(temp1);
	cbn = pvapre.cbn.CbR;
	d_vfn = cnn * cbn * d_vfb;
	//��������/���ϻ�����
	double gp[3] = { 0,0,gravity };
	gl = gp;
	d_vgn = (gl - (omega_ie_n * 2 + omega_en_n) * pvapre.vel) * imucur.dt;
	//�õ��м�ʱ���ٶ�
	midvel = pvapre.vel + (d_vfn + d_vgn) / 2.0;
	//���Ƶõ��м�λ��
	Vector2Quaternion(temp1, qnn);
	double arraytemp[3] = { 0,0,-omega_e * imucur.dt / 2.0 };
	temp2 = arraytemp;
	Vector2Quaternion(temp2, qee);
	_Setqne(pvapre.pos, qne);
	qne = qee * qne * qnn;
	double height = pvapre.pos(2) - midvel(2) * imucur.dt / 2.0;
	midpos = _Setpos(qne, height);
	//���¼����м�ʱ�̵�RM��RN��omega_ie_n��omega_en_n
	RM = _SetRM(midpos(0));
	RN = _SetRN(midpos(0));
	omega_ie_n = _omega_ien(midpos(0));
	omega_en_n = _omega_enn(midpos(0), midpos(2), midvel(0), midvel(1), midvel(2));
	//���¼���nϵ��ƽ������������
	temp3 = (omega_ie_n + omega_en_n) * imucur.dt / 2.0;
	cnn = I33 - Antisymmetry(temp3);
	d_vfn = cnn * cbn * d_vfb;
	//���¼�������/���ϻ�����
	gp[2] = _Setgp(midpos(0), midpos(2));
	gl = gp;
	d_vgn = (gl - (omega_ie_n * 2 + omega_en_n) * midvel) * imucur.dt;
	//�ٶȸ������
	pvacur.vel = pvapre.vel + d_vfn + d_vgn;
	//ʱ��
	pvacur.time = imucur.gt.SecOfWeek;
}

/// <summary>
/// λ�ø���
/// </summary>
void posUpdate(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imupre, _IMU& imucur)
{
	Vector temp1(3), temp2(3), temp3(3), midvel(3), midpos(3);
	Quaternion qne, qnn, qee;
	//���¼����м�ʱ�̵��ٶȺ�λ��
	midvel = (pvacur.vel + pvapre.vel) / 2.0;
	midpos = pvapre.pos + _SetDRi(pvapre.pos) * midvel * imucur.dt / 2.0;
	//_SetDRi(pvapre.pos).Show();
	//���¼����м�ʱ�̵ĵ������
	double RM = 0.0, RN = 0.0;
	Vector omega_ie_n(3), omega_en_n(3);
	RM = _SetRM(midpos(0));
	RN = _SetRN(midpos(0));
	omega_ie_n = _omega_ien(midpos(0));
	omega_en_n = _omega_enn(midpos(0), midpos(2), midvel(0), midvel(1), midvel(2));
	// ���¼��� kʱ�̵�k-1ʱ�� nϵ��תʸ��
	temp1 = (omega_ie_n + omega_en_n) * imucur.dt;
	Vector2Quaternion(temp1, qnn);
	// eϵת����Ч��תʸ�� (k-1ʱ��kʱ�̣�����ȡ����)
	double arraytemp[3] = { 0,0,-omega_e * imucur.dt };
	temp2 = arraytemp;
	Vector2Quaternion(temp2, qee);
	//λ�ø������
	_Setqne(pvapre.pos, qne);
	qne = qee * qne * qnn;
	double height = pvapre.pos(2) - midvel(2) * imucur.dt;
	pvacur.pos = _Setpos(qne, height);


}

/// <summary>
/// ��̬����
/// </summary>
void attUpdate(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imupre, _IMU& imucur)
{
	Quaternion qne_pre, qne_cur, qne_mid, qnn, qbb, qne_temp, qbn_pre;
	Vector temp1(3), midpos(3), midvel(3);
	//RotationVector temp;
	// ���¼����м�ʱ�̵��ٶȺ�λ��
	midvel = (pvapre.vel + pvacur.vel) / 2.0;
	_Setqne(pvapre.pos, qne_pre);
	_Setqne(pvacur.pos, qne_cur);
	qne_temp = qne_cur.Inverse() * qne_pre;
	Quaternion2Vector(qne_temp, temp1);
	//temp1 = temp.phi;
	temp1 = temp1 / 2.0;
	Vector2Quaternion(temp1, qne_temp);
	qne_temp = qne_temp.Inverse();
	qne_mid = qne_pre * qne_temp;
	double height = (pvapre.pos(2) + pvacur.pos(2)) / 2.0;
	midpos = _Setpos(qne_mid, height);
	//���¼����м�ʱ�̵ĵ������
	double RM = 0.0, RN = 0.0;
	Vector omega_ie_n(3), omega_en_n(3);
	RM = _SetRM(midpos(0));
	RN = _SetRN(midpos(0));
	omega_ie_n = _omega_ien(midpos(0));
	omega_en_n = _omega_enn(midpos(0), midpos(2), midvel(0), midvel(1), midvel(2));
	// ����nϵ����ת��Ԫ�� k-1ʱ�̵�kʱ�̱任
	temp1 = (omega_ie_n + omega_en_n) * imucur.dt * (-1);
	Vector2Quaternion(temp1, qnn);
	DCM Cnn;
	Quaternion2DCM(qnn, Cnn);
	// ����bϵ��ת��Ԫ�� ��������Բ׶���
	temp1 = imucur.dtheta + imupre.dtheta * imucur.dtheta / 12.0;
	Vector2Quaternion(temp1, qbb);
	DCM Cbb;
	Quaternion2DCM(qbb, Cbb);

	// ��̬����
	pvacur.qbn = qnn * pvapre.qbn * qbb;
	Quaternion2DCM(pvacur.qbn, pvacur.cbn);
	Quaternion2EulerAngles(pvacur.qbn, pvacur.att);


}

/// <summary>
/// ��е����
/// </summary>
void Mechanican(_IMUPVA& pvapre, _IMUPVA& pvacur, _IMU& imupre, _IMU& imucur)
{
	//// ���ν����ٶȸ��¡�λ�ø��¡���̬����, ���ɵ���˳��
	velUpdate(pvapre, pvacur, imupre, imucur);
	posUpdate(pvapre, pvacur, imupre, imucur);
	attUpdate(pvapre, pvacur, imupre, imucur);

}