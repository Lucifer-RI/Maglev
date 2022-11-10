#pragma once

/* Created by ysfyuan on 2022/10/20 */

#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <stdio.h>
#include <Eigen/Dense>
#include "Feeds.h"
#include "DataFusion.h"
#include "SystemStatus.h"

/* 迭代数据源来自映射文件中的全局变量数据 */

/* 卡尔曼滤波抽象 */
class KF : public DataFusion{
public:
    /* 构造函数 */
    KF(int status_size, int mersure_size, int u_size, int feeds_num, std::string fusion_name, Feeds* pfeeds);
    /* 析构函数 */
    virtual ~KF();
    /* 实际融合迭代函数,继承自DataFusion，动态多态 */
    virtual void RunFunc();
    /* 获取新数据 */
    Eigen::VectorXd GetMeasure();
    /* 融合算法初始化 */
    void InitFusion(Eigen::VectorXd& x, Eigen::MatrixXd& p, 
                    Eigen::MatrixXd& r, Eigen::MatrixXd& q, Eigen::MatrixXd& a,
                    Eigen::MatrixXd& b, Eigen::VectorXd& u, Eigen::MatrixXd& h);
    /* 根据上一次估量值预估状态值 */
    void PredictState();
    /* 预估误差协方差 */
    void PredictCov();
    /* 根据观测值z获取中间计算值 */
    Eigen::VectorXd MeasureState(Eigen::VectorXd& z);
    /* 更新卡尔曼滤波增益 */
    Eigen::MatrixXd UpdateKGain();
    /* 更新系统最优估算值 */
    void UpdateState(Eigen::VectorXd& z);
    /* 更新误差协方差 */
    void UpdateCov();

private:
    /* Runfunc中需要用到的迭代变量 */
    Eigen::VectorXd mX;  /* 输出状态值，以及预测阶段的估计值， n行 */
    Eigen::VectorXd mZ;  /* 系统输入传感器观测值向量， m行 或者 m行m列*/
    Eigen::VectorXd mU;  /* 系统控制向量，暂不考虑 */
    Eigen::MatrixXd mB;  /* 系统控制矩阵，暂不考虑 */
    Eigen::MatrixXd mA;  /* 状态转移矩阵， n行n列 */
    Eigen::MatrixXd mP;  /* 针对系统状态向量（包括中间预测阶段）的系统误差协方差矩阵， n行n列 */
    Eigen::MatrixXd mH;  /* 观测向量到状态向量的转移矩阵，若都是线性恒等，则对角线为1， m行n列 */
    Eigen::MatrixXd mR;  /* 传感器噪声协方差矩阵， m行m列， 参考传感器手册 */
    Eigen::MatrixXd mQ;  /* 过程噪声协方差矩阵， n行n列， 一般设置非常小的值 */
    Eigen::MatrixXd mK;  /* 卡尔曼增益因子 */
    Eigen::MatrixXd mI;  /* 单位矩阵 */

    SystemStatus* pStatus;   /* 系统状态类的指针，指向唯一实例化的SystemStatus对象 */

    Feeds* pFeed;     /* 系统数据源类的指针，用于获取观测数据 */

    int StatusSize;  /* 状态向量x的长度 */
    int MeasureSize; /* 观测向量z的长度 */
    int Usize;       /* 控制矩阵的行数，暂不考虑 */

    long long mMeasureTime;   /* 获取的数据时间戳 */
};
