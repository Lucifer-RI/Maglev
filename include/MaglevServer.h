#pragma once
/* created by ysfyuan on 2022/9/29 */

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <climits>
#include <stdlib.h>
#include "Feeds.h"
#include "DataFusion.h"
#include "KF.h"
#include "SystemStatus.h"
#include "Service.h"

/* MaglevServer类，系统总类，其他模块通过基于对象的方式调用 */
class MaglevServer
{
public:
    MaglevServer(int feeds_num, int services_num, int server_status, int fusions_num);  /* 构造函数 */
    ~MaglevServer(); /* 析构函数 */

    void ServerStart(const std::vector<int>& service_ports, const std::vector<std::string>& service_names, const std::string& fusion_names); /* 系统启动 */
    void ServerEnd();   /* 系统关闭 */
    void FeedsStart();   /* 开启数据源通道 */
    // void FusionStart(DataFusion* pExactlyFusion);  /* 开启数据融合 */
    void ServiceStart(); /* 开启服务 */

private:
    int FeedsNum; /* 数据源数量 */
    int ServicesNum; /* 系统支持服务类型数量 */
    int FusionsNum;  /* 系统支持的融合方式数量 */
    std::vector<Service*> ServicesList; /* 系统的服务列表，列表元素为服务对象 */
    Feeds* pCurFeed;  /* 系统的当前数据源对象 */
    DataFusion* pFusion; /* 系统支持的数据融合方式 */
    int ServerStatus = -1; /* 系统状态标识 */

    SystemStatus* pSysStatus;   

};
