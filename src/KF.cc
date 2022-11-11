/* Created by ysfyuan on 2022/10/21 */

#include "KF.h"


/* 构造函数 */
/* 注意： 抽象基类中没有定义构造函数时不会有默认构造函数，需要在派生类的构造函数初始化列表中显示调用 */
KF::KF(int status_size, int measure_size, int u_size, int feeds_num, std::string fusion_name, Feeds* pfeeds)
    : StatusSize(status_size), MeasureSize(measure_size), Usize(u_size), DataFusion(feeds_num, fusion_name)
{
    pStatus = SystemStatus::GetInstance();  /* 将系统状态改成单例模式，在此处调用GetInstance */
    FusionStatus = 1;  /* 融合状态标志 */
    pFeed = pfeeds;
}


/* 析构函数 */
KF::~KF()
{
    pStatus = nullptr; /* 不需要在此处调用delete释放，因为对象为系统状态的唯一实例化 */
}


void KF::RunFunc()
{
    /* 初始化卡尔曼滤波的过程矩阵 */
    Eigen::VectorXd x;
    Eigen::MatrixXd p;
    Eigen::MatrixXd q;
    Eigen::MatrixXd r;
    Eigen::MatrixXd a;
    Eigen::MatrixXd b;
    Eigen::MatrixXd h;
    Eigen::VectorXd z;
    Eigen::VectorXd u;

    x.resize(StatusSize);
    x.setZero();

    /* 协方差 */
    p.resize(StatusSize, StatusSize);
    p.setZero();
    p.setIdentity();
    p(0,1) = 10;
    p(0,2) = 10;
    p(1,2) = 10;
    /* 协方差表示的是两个变量的总体的误差，如果两个变量的变化趋势一致，
    也就是说如果其中一个大于自身的期望值，另外一个也大于自身的期望值，
    那么两个变量之间的协方差就是正值。 
    如果两个变量的变化趋势相反，即其中一个大于自身的期望值，
    另外一个却小于自身的期望值，那么两个变量之间的协方差就是负值。
    从数值上来看,协方差数值越大,两个变量同向的程度越大,反之亦然.
    而方差是协方差的一种特殊情况，即当两个变量是同一变量(x和x)。
    我们知道方差它反映的是数据序列与均值的关系。  */

    /* 过程噪声 */
    q.resize(StatusSize, StatusSize);
    q.setZero();
    for(int i = 0; i < StatusSize; ++i)
    {
        q(i,i) = 1e-5;
    }

    /* 传感器噪声协方差矩阵 */
    r.resize(MeasureSize, MeasureSize);
    r.setZero();
    r(0,0) = 0.1;
    r(1,1) = 0.1;
    r(2,2) = 0.1;

    /* 状态转移矩阵 */
    a.resize(StatusSize, StatusSize);
    a.setIdentity();
    /* 可扩展为非线性,即丰富该转移矩阵为Jacobi矩阵 */

    /* 控制输入加权矩阵 */
    b.resize(StatusSize, Usize);
    b.setIdentity();

    /* 观测矩阵到状态矩阵的转换矩阵 */
    h.resize(MeasureSize, StatusSize);
    h.setIdentity();

    z.resize(MeasureSize);

    u.resize(Usize);
    u.setZero();

    InitFusion(x, p, r, q, a, b, u, h);
    
    /* 初始化系统状态结构体 */
    StatusData* pStatusData = new StatusData();
    memset(pStatusData, 0, sizeof(StatusData));
    pStatusData->StatusNum = 3;
    
    /* 此处需考虑使用事件驱动或者周期驱动方式 */
    while(FusionStatus)
    {
        /* 多通道传感器观测值获取 */
        z = GetMeasure();
        /* 根据上次估值预测 */
        PredictState();
        /* 估算COV */
        PredictCov();
        /* 更新Gain、X*/
        UpdateState(z);
        /* 更新COV */
        UpdateCov();
        /* 将迭代后最优估计值更新到status中 */
        pStatusData->CurPos = mX(0);
        pStatusData->CurSpeed = mX(1);
        pStatusData->CurAcc = mX(2);
        pStatusData->CurTime = mMeasureTime;  /* 观测时间 */

        /* 驱动状态更新 */
        pStatus->UpdateStatus(pStatusData);
    }

}


/* TODO: GetMeasure实现多通道传感器观测值获取*/ 
Eigen::VectorXd KF::GetMeasure()
{
    /* 静态构造原始数据结构体 */
    RawData NewData;
    memset(&NewData, 0, sizeof(RawData));
    int ret = pFeed->ReadData(&NewData);
    /* ret 可用于调节滤波系统的数据获取筛选 */
    Eigen::VectorXd MeasureData;
    MeasureData.resize(MeasureSize);
    /* 获取有限观测数据 */
    /* Magnet需要转化成常规pos数据和speed数据 */
    std::pair<int,int> MagData = PosGetFunc(pFeed);
    /* Pos */
    if(NewData.RFIDFlag == 1)
    {
        MeasureData(0) = NewData.RFIDpos;
    }
    else
    {
        MeasureData(0) = MagData.first;
    }

    /* Speed */
    /* 存在多种计算方式，更据前一次IMU数据积分，根据Pos求导 */
    /* 通过Magnet历史数据来获取速度 */
    // MeasureData(1) = NewData.IMU * 
    MeasureData(1) = MagData.second;
    /* Acc */
    MeasureData(2) = NewData.IMU;

    /* Time */
    mMeasureTime = NewData.RawTime; 
    return MeasureData;
}


/* KF初始化 */
void KF::InitFusion(Eigen::VectorXd& x, Eigen::MatrixXd& p, 
                    Eigen::MatrixXd& r, Eigen::MatrixXd& q, Eigen::MatrixXd& a,
                    Eigen::MatrixXd& b, Eigen::VectorXd& u, Eigen::MatrixXd& h)
{
    std::cout << "FUsion(KF)  Initial ... " << std::endl;
    /* 矩阵向量初始化 */
    mX.resize(StatusSize);
    mX.setZero();
    mX = x;

    mU.resize(Usize);
    mU.setZero();
    mU = u;

    mB.resize(StatusSize, Usize);
    mB.setZero();
    mB = b;
    /* 暂不考虑输入控制 */

    mZ.resize(MeasureSize);
    mZ.setZero();

    mA.resize(StatusSize,StatusSize);
    mA.setZero();
    mA = a;

    mP.resize(StatusSize, StatusSize);
    mP.setZero();
    mP = p;

    mH.resize(MeasureSize, StatusSize);
    mH.setZero();
    mH = h;

    mR.resize(MeasureSize, MeasureSize);
    mR.setZero();
    mR = r;

    mQ.resize(StatusSize, StatusSize);
    mQ.setZero();
    mQ = q;

    mI.resize(StatusSize, StatusSize);
    mI.setIdentity();

    std::cout << "Initial Completed ! " << std::endl;

}


/* PredictState ，根据上次最佳估计值估算当前状态 */
void KF::PredictState()
{
    /* X(k/k-1) = A*X(k-1/k-1) + B*U */
    Eigen::VectorXd temp_state = mA*mX + mB*mU;
    mX = temp_state;
}


/* PredictCov, 估算误差协方差 */
void KF::PredictCov()
{
    /* P(k/k-1) = A*P*A(T) + Q */
    Eigen::MatrixXd temp_cov =  mA*mP*(mA.transpose()) + mQ;
    mP = temp_cov;
}


/* MeasureState, 根据观测值计算中间量 */
Eigen::VectorXd KF::MeasureState(Eigen::VectorXd& z)
{
    mZ = z;
    Eigen::VectorXd temp_res = mZ - mH*mX;
    return temp_res;
}


/* 更新卡尔曼滤波增益 */
Eigen::MatrixXd KF::UpdateKGain()
{
    /* g(k) = P(k/k-1)*H(T)/(H*P(k/k-1)*H(T) + R) */
    Eigen::MatrixXd temp_gain = mP*(mH.transpose())*((mH*mP*(mH.transpose()) + mR).inverse());  
    return temp_gain;
}


/* 更新系统最优估算值 */
void KF::UpdateState(Eigen::VectorXd& z)
{
    Eigen::MatrixXd kal_gain = UpdateKGain();
    Eigen::VectorXd mea_res = MeasureState(z);
    /* X(k/k) = X(k/k-1) + g(k)*(Z - H*X(k/k-1)) */
    mX = mX + kal_gain * mea_res;
}


/* 更新误差协方差 */
void KF::UpdateCov()
{
    Eigen::MatrixXd kal_gain = UpdateKGain();
    Eigen::MatrixXd temp_mat = kal_gain * mH;
    mP = (mI - temp_mat) * mP;
}


