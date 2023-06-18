#include <iostream>
#include <map>
#include"IMU.h"
#include"INS.h"
#include"GNSS.h"
#include"EKF.h"


double initialPos[3] = { 30.5278108948,114.3557126173,22.320 };//初始位置
double initialVel[3] = { 0.000, 0.000,0.000 };               //初始速度
//double initialPos[3] = {30.5282960229,114.3557510106,23.141};//初始位置
//double initialVel[3] = { 0.000, 0.000,0.000 };               //初始速度
double std_pos[3] = { 0.008,0.008,0.021 };                        //单位m
double std_vel[3] = { 0.000,0.000,0.000 };                               //单位m/s

EulerAngles initialAtt;                                      //初始姿态
double std_att[3] =
            {deg2rad(0.05),deg2rad(0.05), deg2rad(0.05)     };     //单位rad
double arrayL_GNSS[3] = { -0.1000,0.2350,-0.8900 };          //GNSS杆臂
//IMU传感器误差方差
double ARW = deg2rad(0.2) / 60.0;                           //deg / \sqrth //rad/sqrts
double VRW = 0.4 / 60.0;                                    //m / s / \sqrth //m / s / \sqrts
double Sigma_bg = deg2rad(24) / 3600.0;                    // deg / h
double Sigma_ba =  400 * 1e-5;                                // mGal //m/s^2
double Sigma_Sg = 1000 * 1e-6;                               // ppm
double Sigma_Sa = 1000 * 1e-6;                               // ppm
using namespace std;
int main()
{
    /* 1.数据读取
    *  1.1 IMU数据读取  
    *  1.2 GNSS数据读取
    *  1.3 IMU初始对准
    *  1.4 IMU 数据及解算结果保存
    */
    char imuFile[] = "data\\wide\\wide.ASC";
    map<double, _IMU> mapImu;
    ReadIMUData(imuFile, mapImu);
    char gnssFile[] = "data\\wide\\wide.pos";
    map<double, _GNSS> mapGnss;
    ReadGNSSData(gnssFile, mapGnss);
    initialAtt = initialAlignment(mapImu, initialPos);     //初始对准
    cout << initialAtt;
    //结果保存
    //ofstream imuData("data\\wide\\wide.txt");
    ofstream insResult("data\\wide\\wide.sins");
    /* 2.起算量设置
    *  2.1 杆臂
    *  2.2 初始位置，速度，姿态
    *  2.3 2.2的标准差
    *  2.4 IMU的初始误差
    */
    Vector L_GNSS(3); L_GNSS = arrayL_GNSS;             //GNSS 杆臂
    _IMUPVA pvapre;                                     //上时刻位速姿
    _IMUPVA pvacur;                                     //本时刻位速姿
    _IMU imupre;                                        //上时刻IMU输出
    _IMU imucur;                                        //本时刻IMU输出
    pvapre.time = mapImu.begin()->second.gt.SecOfWeek + toa;
    initialPos[0] = deg2rad(initialPos[0]);initialPos[1] = deg2rad(initialPos[1]);  
    pvapre.pos = initialPos;
    pvapre.vel = initialVel;
    pvapre.att = initialAtt; 
    EulerAngles2DCM(pvapre.att, pvapre.cbn); 
    EulerAngles2Quaternion(pvapre.att, pvapre.qbn);
    pvapre.velpre = initialVel;
    _IMUError imuerror;                                 //IMU 初始误差

    /* 3.中间量的设置
    *  3.1 kalman滤波状态向量及协方差阵
    *  3.2 GNSS数据迭代器
    */
    //map<double, _IMUPVA> mapPva;
    //mapPva[pvapre.time] = pvapre;

    Vector xk(21);//状态向量

    Matrix qt(18, 18);//系统噪声协方差阵qt
    _Setq(qt, VRW, ARW, Sigma_bg, Sigma_ba, Sigma_Sg, Sigma_Sa, tau);
    Matrix Pk(21, 21);//初始协方差矩阵
    _SetP(Pk, std_pos, std_vel, std_att, Sigma_bg, Sigma_ba, Sigma_Sg, Sigma_Sa);      //初始协方差阵  


    auto itGnss = mapGnss.begin();
    while (itGnss->first - pvapre.time < EPS)
        itGnss++;
    double updatetime = itGnss->first;     //GNSS开始更新时间
    //auto itEnd = mapImu.end();
    /* 4.SINS_LC
    *  4.1 机械编排
    *  4.2 以GNSS数据为量测更新
    *  4.3 闭环反馈
    */

    //ofstream time("data\\wide\\time.txt");

    for (auto it = mapImu.begin(); it != mapImu.end(); it++)
    {
        //imuData << it->second;
        if (it->first - pvapre.time < EPS) { imupre = it->second; imupre.dt = 1.0 / dataRate; continue; }
        imucur = it->second;
        imucur.dt = 1.0 / dataRate;
        int res = isToUpdate(imupre.gt.SecOfWeek, imucur.gt.SecOfWeek, updatetime);
        bool isZUPT = isZeroSpeed(it,mapImu.end());
        //res = 0;
        if (res == 0) 
        {
            KF_Predict(pvapre, pvacur, imupre, imucur, imuerror, qt, Pk, xk);
            //进行零速更新
            if (isZUPT)
            {
                KF_Update_ZUPT(pvacur, imucur, Pk, xk);
                stateFeedback(pvacur, imucur, xk, imuerror);
            }
        }
        else if (res == 3) //不需要内插
        {
            KF_Predict(pvapre, pvacur, imupre, imucur, imuerror, qt, Pk, xk);
            KF_Update(itGnss->second, pvacur, imucur, L_GNSS, Pk, xk);
            stateFeedback(pvacur, imucur, xk, imuerror);
            itGnss++;
            if (itGnss == mapGnss.end())
                itGnss--;
            else
                updatetime = itGnss->first;

        }
        else   //需要内插
        {
            _IMU imumid;
            /* GNSS时刻夹在IMU前后两时刻之间
            * GNSS同步时刻的IMU增量
            * 内插GNSS时刻的增量
            */
            imuInterpolate(imupre, imucur, updatetime, imumid);
            // t-1时刻到GNSS更新时刻 一步预测
            KF_Predict(pvapre, pvacur, imupre, imumid, imuerror, qt, Pk, xk);
            //使用GNSS时刻的pva以及GNSS数据对此时刻的量测进行更新
            KF_Update(itGnss->second, pvacur, imumid, L_GNSS, Pk, xk);
            //闭环反馈
            stateFeedback(pvacur, imucur, xk, imuerror);

            //更新GNSS数据
            itGnss++;
            if (itGnss == mapGnss.end())
                itGnss--;
            else
                updatetime = itGnss->first;

            //计算GNSS时刻到当前时刻的pva，进行状态传播
            pvapre = pvacur;
            KF_Predict(pvapre, pvacur, imumid, imucur, imuerror, qt, Pk, xk);

        }

        pvapre = pvacur;
        imupre = imucur;
        //cout << pvacur;
        insResult << pvacur;
        
    }

    /* 5.关闭文件
    */
    insResult.close();
    //errResult.close();
    //imuData.close();
    //time.close();
}

