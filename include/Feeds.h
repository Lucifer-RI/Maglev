#pragma once 

/* created by ysfyuan on 2022/9/30 */

#include <vector>
#include <iostream>
#include <thread>
#include <string>
#include <algorithm>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory.h>
#include <string.h>
#include <time.h>


/* 数据源接收到的原生数据 */
struct RawData {
    int Magnet;  /* TMR磁场强度观测值 */
    int MagnetFlag; /* TMR传感器通道接收数据标识 */
    int IMU;    /* IMU传感器的方向加速度 */
    int IMUFlag;    /* IMU传感器通道接受数据标识 */
    int RFIDpos;    /* RFID传感器的定位观测值 */
    int RFIDFlag;   /* RFID传感器通道接受数据标识 */
    long long RawTime;    /* 记录每一次的数据采集驱动时间 */

};


class Feeds{
public:
    Feeds(long long cur_time, int feeds_status);    /* 数据源Feeds对象的构造函数声明 */
    ~Feeds(); /* 析构函数 */

    int FeedsOpen();  /* 数据通道开启函数，开始接收数据，设置返回值为int类型用于判断数据中断原因 */
    
    std::pair<int,int> RenewParser(char buf[], int Length);   /* 串口数据协议解析 */
    /*  */
    /*传感器数据更新，判断传感器数据类型，返回对应传感器数值*/
    
    // int DataParser(RawData* pData);  /* 用于解析一组RawData，提供给Fusion迭代计算使用 */

    int MagParser(std::string& MagData);  /*  */

    int IMUParser(std::string& IMUData);

    int RFIDParser(std::string& RFIDData);

    int GetReadIndex();   /* 获取共享文件中的读标志位内容 */
    void UpdateReadIndex(); /* 更新读标志位 */

    int GetWriteIndex();  /* 获取共享文件中的写标志位内容 */
    void UpdateWriteIndex(); /* 更新写标志位 */

    void WriteData(RawData* pData); /* 向共享文件中写入数据 */
    int ReadData(RawData* pDataOut);   /* 从共享文件中读出最新传感器数据 */

private:
    int FeedsStatus; /* 信号源标识，如有三路信号源则有6种状态 */
    long long CurTime;  /* 当前的事件驱动时间戳 */
    
    /* 串口通信用到的文件描述符 */
    int mFdMag;
    int mFdIMU;
    int mFdRFID;
    // 用落地到mmap的方式去进行IPC，开启一个足量空间存储RawData
    // 存储内存前用两个读写标志去标记下标
    void* mAddr; /* 共享文件初始地址 */
    RawData* mStartAddr;  /* RawData数据初始地址 */
    int mFdFile;  /* 用于存放共享文件的文件描述符 */
    int mWriteIndex;  
    int mReadIndex; 
};