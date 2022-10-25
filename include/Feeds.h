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



/* 数据源接收到的原生数据 */
struct RawData {
    double Magnet;  /* TMR磁场强度观测值 */
    int MagnetFlag; /* TMR传感器通道接收数据标识 */
    double IMUx;    /* IMU传感器的X方向加速度 */
    double IMUy;    /* IMU传感器的Y方向加速度 */
    double IMUz;    /* IMU传感器的Z方向加速度 */
    int IMUFlag;    /* IMU传感器通道接受数据标识 */
    double RFIDpos;    /* RFID传感器的定位观测值 */
    int RFIDFlag;   /* RFID传感器通道接受数据标识 */

    int RawTime;    /* 记录每一次的数据采集驱动时间 */

};


class Feeds{
public:
    Feeds(int cur_time, int feeds_status);    /* 数据源Feeds对象的构造函数声明 */
    ~Feeds(); /* 析构函数 */

    int FeedsOpen();  /* 数据通道开启函数，开始接收数据，设置返回值为int类型用于判断数据中断原因 */

    int DataParser(RawData* pData);  /* 用于解析一组RawData，提供给Fusion迭代计算使用 */

private:
    int FeedsStatus; /* 信号源标识，如有三路信号源则有6种状态 */
    int CurTime;  /* 当前的事件驱动时间戳 */
    RawData* pRawData; /* 存储当前驱动周期获取的观测数据值 */
    
    // 用落地到mmap的方式去进行IPC，开启一个足量空间存储RawData
    // 存储内存前用两个读写标志去标记下标
    int WriteIndex;  
    int ReadIndex; 
};