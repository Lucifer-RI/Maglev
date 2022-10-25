#pragma once

/* created by ysfyuan on 2022/9/30 */

#include <vector>
#include <string>
#include <stdlib.h>
#include <iostream>
#include <thread>
#include <unistd.h>


/* 融合方式抽象类，作为基类，不可实例化，用于定义需要实现的接口以及部分属性 */
class DataFusion{
public:
    DataFusion(const int feeds_num, const std::string fusion_name);  /* 构造函数 */

    virtual ~DataFusion();     /* 析构函数 */

    virtual void RunFunc() = 0;    /* 纯虚函数用于定义接口，在该函数中使用当前的系统状态值以及观测值去进行融合运算 */
    
    int FusionStatus; /* 该融合方式的状态 */ 

private:
    int FeedsNum;  /* 传感器数据通道数 */
    
    std::string FusionName;  /* 融合方式名称 */

    int FusionID;  /* 该融合方式在服务器中的序号 */

};

