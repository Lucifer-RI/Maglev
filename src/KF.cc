/* Created by ysfyuan on 2022/10/21 */

#include "KF.h"


/* 构造函数 */
/* 注意： 抽象基类中没有定义构造函数时不会有默认构造函数，需要在派生类的构造函数初始化列表中显示调用 */
KF::KF(int status_size, int measure_size, int u_size, int feeds_num, std::string fusion_name, Feeds* pfeeds)
    : StatusSize(status_size), MeasureSize(measure_size), Usize(u_size), DataFusion(feeds_num, fusion_name)
{
    pStatus = SystemStatus::GetInstance();  /* 将系统状态改成单例模式，在此处调用GetInstance */
    FusionStatus = 1;  /* 融合状态标志 */
    ValidMeasureFlag = 0;  /* 初始化状态为0， 有效态为1， 无效态为-1 */
    pFeed = pfeeds;
    MeasureLength = 50;
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

    int RFID_Correction = 0;
    
    /* 此处需考虑使用事件驱动或者周期驱动方式 */
    while(FusionStatus)
    {
        /* 多通道传感器观测值获取 */
        usleep(200000);
        z = GetMeasure(pStatusData, RFID_Correction);
        if(ValidMeasureFlag == -1)
        {
            continue;
        }
        /* 如果存在RFID信号，则加入紧耦合进行累计误差修正 */
        if(RFID_Correction == 1)
        {
            pStatusData->CurPos = z(0);
            RFID_Correction = 0;
            // InitFusion(z, p, r, q, a, b, u, h);
            mX = z;
            mP = p;
            continue;
        }
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
        // std::cout << "RFIF_Correction :  "<< RFID_Correction << std::endl;
        /* 驱动状态更新 */
        pStatus->UpdateStatus(pStatusData);
    }
}


/* 区间误差消除 */
void KF::PosCorrection(int CorrectPos)
{
    int StartIndex = pFeed->GetReadIndex() - MeasureLength;
    if(StartIndex < 0)
    {
        StartIndex = 0;
    }
    int EndIndex = pFeed->GetReadIndex();
    RawData* pData = pFeed->mStartAddr;
    int Distance = 3500;
    int index_correct = 0;
    for(int i = EndIndex; i >= StartIndex; --i)
    {
        if((pData+i)->MagMax == 1)
        {
            /* 该采样数据为波峰 */
            if(CorrectPos > (index_correct * Distance))
            {
                (pData+i)->Pos = CorrectPos - (index_correct * Distance);
            }
            else
            {
                (pData+i)->Pos = 0;
            }
            ++index_correct;
            std::cout <<"+" <<  (pData+i)->Pos / 10000 << ", "; 
        }
        else if((pData+i)->MagMax == -1)
        {
            /* 该采样数据为波峰 */
            if(CorrectPos > (index_correct * Distance))
            {
                (pData+i)->Pos = CorrectPos - (index_correct * Distance);
            }
            else
            {
                (pData+i)->Pos = 0;
            }
            ++index_correct;
            std::cout << "-" << (pData+i)->Pos / 10000 << ", "; 
        }
    }
    std::cout << std::endl; 
}


/* 从Magnet相关历史数据（离散信号值）计算峰值点，从而获取Pos以及speed信息 */
/* TODO : 待验算证明 */
void KF::PosGetFunc(StatusData* CurStatus, Feeds* pfeed, int Length, std::pair<uint64_t,int>& res, int& confident_flag, long long MeasureTime)
{
    // 峰值间距（磁轨长度）
    uint64_t distance = 7000; /* 70cm的理想峰值间距 */
    std::vector<int> signal(2,0);
    uint16_t flag;
    /* 
       用于纪录变化信息的数组，
       其中数组中元素为1则说明当前点相较前一元素为强度升高
       数组中元素为-1则说明当前点相较前一元素为强度降低
    */
    std::vector<int> sign; 

    std::vector<int> indMax;  
    std::vector<int> indMin;  
    /* 获取读标志位 */
    uint16_t HasReadIndex = pFeed->GetReadIndex() - 1;
    /* 获取检测数据队列的队首地址 */
    // std::cout << "HasReadIndex : " << HasReadIndex << std::endl;
    int RealProcessLength = 0;
    if(HasReadIndex <= Length)
    {
        RealProcessLength = HasReadIndex;
    }
    else
    {
        RealProcessLength = Length;
    }
    
    RawData* StartPos = pFeed->mStartAddr + (HasReadIndex - RealProcessLength);
    
    std::vector<int> indexVec;
    // std::cout << "Catch High & Los" << std::endl;
    for(int i = 0; i < RealProcessLength; i++)  
    {  
        /* 取出当前最新的Length个带Magnet数据的Rawdata */
        // std::cout << (StartPos+i)->MagnetFlag << " . ";
        if(!(StartPos+i)->MagnetFlag)
        {
            continue;
        }
        /* 将有效数据下标纪录在数组中 */
        indexVec.emplace_back(i);
        /* 有效数据小于2则不进行判断 */
        if(indexVec.size() < 2)
        {
            continue;
        }
        
        int diff = ((StartPos+i)->Magnet) - ((StartPos+indexVec[indexVec.size()-2])->Magnet);  
        // std::cout << diff << " , ";
        if(diff > 500000)  
        {  
            if(!flag)
            {
                signal[1]++;
            }
            sign.emplace_back(1);  
        }  
        else if(diff< -500000)  
        { 
            if(!flag )
            {
                signal[0]++;
            } 
            sign.emplace_back(-1);  
        }  
        else  
        {  
            sign.emplace_back(0);  
        }
        if(signal[0] > 0 && signal[1] > 0 ) 
        {
            flag = 1;
        }
    } 
    for(int j = 1; j < sign.size(); j++)  
    {   
        int diff_max = sign[j]-sign[j-1]; 
        if(diff_max > 0)  
        { 
            std::cout <<"+ : " << indexVec[j] + (HasReadIndex - RealProcessLength) << std::endl; 
            indMax.emplace_back(indexVec[j]);
            (StartPos+indexVec[j])->MagMax = 1;
        }  
        else if(diff_max < 0)  
        {  
            std::cout <<"- : " << indexVec[j] + (HasReadIndex - RealProcessLength) << std::endl; 
            indMin.emplace_back(indexVec[j]);
            (StartPos+indexVec[j])->MagMax = -1;
        }  
    }
      
    /*  加上第一个波峰所在采样数据的位置信息，以及波峰波谷计算的相互验证 */
    int MaxLen = indMax.size();
    int MinLen = indMin.size();

    if(flag == 0 || (MaxLen < 2 && MinLen < 2))  /* 如果是无效数据，则找出上一个波峰的位置速度信息，进行叠加运算 */
    {
        confident_flag = 0;
        /* 上一个波峰位置信息 */
        // if(MaxLen < 2)
        // {
        //     res.first = 0;
        //     res.second = 0;
        //     std::cout << "Has not Enough Maxdata for Mag" << std::endl;
        //     return ; 
        // }
        std::cout << "Has Not Confident Maxdata for Mag" << std::endl;
        int posLast = CurStatus->CurPos;
        int speedLast = CurStatus->CurSpeed;
        res.first = CurStatus->CurPos + CurStatus->CurSpeed * (MeasureTime - CurStatus->CurTime);
        res.second = speedLast;
        (pFeed->mStartAddr+HasReadIndex)->Pos = res.first;
        (pFeed->mStartAddr+HasReadIndex)->Speed = res.second;
        /* 将速度和位置信息赋给最后一个波峰或者波谷 */
        if(MaxLen > 1 && MinLen > 1)
        {
            (StartPos+std::max(indMax.back(), indMin.back()))->Pos = res.first;
            (StartPos+std::max(indMax.back(), indMin.back()))->Speed = res.second;
        }
        else if(MaxLen > 1)
        {
            (StartPos+indMax.back())->Pos = res.first;
            (StartPos+indMax.back())->Speed = res.second;
        }
        else if(MinLen > 1)
        {
            (StartPos+indMin.back())->Pos = res.first;
            (StartPos+indMin.back())->Speed = res.second;
        }
        return;
    }

    /* 上一个波峰位置信息 */
    /* TODO: 校验逻辑 */
    if(MinLen >= 2)
    {
        /* 扫描到两个以上波谷 */
        int posLastMin = (StartPos+indMin[MinLen - 2])->Pos;
        int posMin = distance + posLastMin;  /* posMax为根据前一个波谷来计算最近波峰的位置 */
        int posPre = posLastMin;  /* posPre 为根据前一个b波谷和当前速度来计算的当前的位置 */
        if(MeasureTime > (StartPos+indMin[MinLen - 2])->RawTime)
        {
            /* posLastMin为利用前一个波谷的位置信息以及那时的速度信息计算的当前位置信息，
            用作可靠性比较 */
            posLastMin += CurStatus->CurSpeed * (MeasureTime - (StartPos+indMin[MinLen - 2])->RawTime);
            posPre += (StartPos+indMin[MinLen - 2])->Speed * (MeasureTime - (StartPos+indMin[MinLen - 2])->RawTime);
        }
        std::cout << " **********************波谷 MagCompare ******************"<< std::endl;
        std::cout << posLastMin << " : " << posPre << " : " << posMin << std::endl;
        
        res.first = PosCompare(posMin, posPre, posLastMin);
    }

    if(MaxLen >= 2)
    {
        /* 扫描到两个以上波峰 */
        int posLastMax = (StartPos+indMax[MaxLen - 2])->Pos;
        int posMax = distance + posLastMax;  /* posMax为根据前一个波峰来计算最近波峰的位置 */
        int posPre = posLastMax;  /* posPre 为根据前一个波峰和当前速度来计算的当前的位置 */
        if(MeasureTime > (StartPos+indMax[MaxLen - 2])->RawTime)
        {
            /* posLastMax为利用前一个波峰的位置信息以及那时的速度信息计算的当前位置信息，
            用作可靠性比较 */
            posLastMax += CurStatus->CurSpeed * (MeasureTime - (StartPos+indMax[MaxLen - 2])->RawTime);
            posPre += (StartPos+indMax[MaxLen - 2])->Speed * (MeasureTime - (StartPos+indMax[MaxLen - 2])->RawTime);
        }
        std::cout << " **********************波峰 MagCompare ******************"<< std::endl;
        std::cout << posLastMax << " : " << posPre << " : " << posMax << std::endl;
        
        res.first = PosCompare(posMax, posPre, posLastMax); 
    }
    // int posMax = distance + posLast;
    // int posMin = distance + posLast;
    // /* 利用上一个波峰的位置信息以及速度信息计算当前最新波峰的位置信息，用作可靠性比较 */
    // posLast += (StartPos+indMax[MaxLen - 2])->Speed * ((StartPos+indMax[MaxLen - 1])->RawTime - (StartPos+indMax[MaxLen - 2])->RawTime);
    // res.first = PosCompare(posMax, posMin, posLast);

    /* 将位置信息计算值赋给结构体存放在mmap中 */
    if(MaxLen > 0 && MinLen > 0)
    {
        if(indMax.back() > indMin.back())
        {
            (StartPos + indMax.back())->Pos = res.first;
        }
        else
        {
            (StartPos + indMin.back())->Pos = res.first;
        }
    }
    else if(MaxLen == 0 || MinLen > 0)
    {
        (StartPos + indMin.back())->Pos = res.first;
    }
    else if(MinLen == 0 || MaxLen > 0)
    {
        (StartPos + indMax.back())->Pos = res.first;
    }

    /* 速度计算方案 */
    if(MaxLen >= 2)
    {
        /* 根据时间求速度 */
        /* 取前一个波峰的位置和时间，当前的时间和位置结果 */
        res.second = (res.first - (StartPos + indMax[MaxLen-2])->Pos) / (MeasureTime - (StartPos + indMax[MaxLen-2])->RawTime);
    }
    else
    {
        res.second = (res.first - (StartPos + indMin[MinLen-2])->Pos) / (MeasureTime - (StartPos + indMin[MinLen-2])->RawTime);
    }
    /* 将速度信息计算值赋给结构体存放在最后一个波峰或者波谷mmap中 */
    if(MaxLen > 0 && MinLen > 0)
    {
        (StartPos + std::max(indMax.back(), indMin.back()))->Speed = res.second;
    }
    else if(MaxLen > 0 )
    {
        (StartPos + indMax.back())->Speed = res.second;
    }
    else
    {
        (StartPos + indMin.back())->Speed = res.second;
    }
    
    (pFeed->mStartAddr+HasReadIndex)->Pos = res.first;
    (pFeed->mStartAddr+HasReadIndex)->Speed = res.second;
    std::cout << "Pos : " << res.first << " Speed : " << res.second << std::endl;
    return;
}


/* Pos 的 权重比较函数，取最高可行度的数据 */
int KF::PosCompare(int pos1, int pos2, int pos3)
{
    // if(posLast >= posMin && posLast >= posMax && posLast%50000 == 0)
    // {
    //     return posLast;
    // }

    // if(posMax == posMin)
    // {
    //     return posMax/2;
    // }
    if(pos2 == 0 && pos3 == 0)
    {
        return pos1;
    }

    int distance = 7000;
    int errorPMin =  abs(pos2-pos1);
    int errorPMax =  abs(pos3-pos1);


    if (errorPMin > distance || errorPMax > distance)
    {
        return pos1;
    }
    else if(errorPMin < errorPMax)
    {
        return pos2;
    }
    return pos3;
}


/* GetMeasure实现多通道传感器观测值获取*/ 
Eigen::VectorXd KF::GetMeasure(StatusData* CurStatus, int& RFID_Correction)
{
    /* 静态构造原始数据结构体 */
    RawData NewData;
    memset(&NewData, 0, sizeof(RawData));
    
    /* ret 可用于调节滤波系统的数据获取筛选 */
    Eigen::VectorXd MeasureData;
    MeasureData.resize(MeasureSize);
    //std::cout << "NewData: " << NewData.RFIDFlag << std::endl;
    int ret = pFeed->ReadData(&NewData);
    //std::cout<< "Measure Read : " << NewData.RawTime <<std::endl;
    if(ret == -1)
    {
        /* 暂时还无采集数据 */
        ValidMeasureFlag = -1;
        std::cout <<  "Initail has not enough data ! " << std::endl;
        return MeasureData;
    }
    /* 返回值不为-1，则为有效观测原始数据值 */
    ValidMeasureFlag = 1;
    // std::cout << " NewData:RFIDflag : " << NewData.RFIDFlag << std::endl;
    if(NewData.RFIDFlag == 1)
    {
        /* RFID : Pos & Speed */
        /* 通过历史数据来获取速度 */
        RFID_Correction = 1;
        MeasureData(0) = NewData.RFIDpos;
        if(NewData.RawTime == CurStatus->CurTime)
        {
            MeasureData(1) = CurStatus->CurSpeed;
        }
        else
        {
            MeasureData(1) = (NewData.RFIDpos - CurStatus->CurPos) / (NewData.RawTime - CurStatus->CurTime);
        }
        /* 区间误差消除,利用RFID的绝对位置信息，对后续会参与运算的前段波峰位置信息进行误差修正 */
        PosCorrection(NewData.RFIDpos);
        NewData.RFIDFlag = 2;
    }
    else
    {
        /* 获取有限观测数据 */
        /* Magnet需要转化成常规pos数据和speed数据 */
        /* 其中pos使用uint64_t, 来防止溢出 */
        std::pair<uint64_t, int> MagData;
        int Confident_flag = 1;  /* 用于标志本次观测生成数据是否为有效值 */
        // std::cout << "PosGETfUNC !" << std::endl;
        PosGetFunc(CurStatus, pFeed, MeasureLength, MagData, Confident_flag, NewData.RawTime);
        /* 若MagData为{-1，-1}， 则说明为无效数据段，则没有新的有效pos数据和speed数据 */
        if(Confident_flag == 0)
        {
            std::cout << "Got no New Pos & Speed, Use last Data !!!" << std::endl;
        }
        /* Mag : Pos & Speed */
        /* 存在多种计算方式，更据前一次IMU数据积分，根据Pos求导 */
        /* 通过Magnet历史数据来获取速度 */
        MeasureData(0) = MagData.first;
        MeasureData(1) = MagData.second;
    }

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
    // std::cout << "Fusion(UKF)  Initial ... " << std::endl;
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

    // std::cout << "Initial Completed ! " << std::endl;

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


