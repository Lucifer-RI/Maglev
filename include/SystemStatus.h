#pragma once

/* created by ysfyuan on 2022/10/6 */

#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <string.h>
#include <sys/mman.h>
#include <sys/types.h>
#include "ServiceClient.h"



class SystemStatus{
public:
    void UpdateStatus(StatusData* pStatus); /* 更新SystemStatus */

    int DelClient(ServiceClient* del_client);   /* 从订阅列表中删除对应客户，返回操作状态码 */

    int AddClient(ServiceClient* new_client);  /* 增加订阅客户，返回ClientID */

    void StatusMonitor();  /* TODO:用于无客户端时，开发者监控系统状态 */

    void NotifyClients();  /* 遍历客户订阅列表，调用每个抽象客户类的publish函数进行推送 */

    
    static SystemStatus* GetInstance() 
    /* SystemStatus设置为单例模式，使用GetInstance（）获取实例 */
    {
        if(StatusInstance == nullptr)
        {
            StatusInstance = new SystemStatus(3);
        }
        return StatusInstance;
    }


private:
    SystemStatus(int status_num = 3);  /* 构造函数 */

    ~SystemStatus();  /* 析构函数 */

    SystemStatus(const SystemStatus&);

    SystemStatus operator=(const SystemStatus&);

    static SystemStatus* StatusInstance;   /* 唯一实例 */

    int StatusNum;  /* 目前关注的系统状态数量 */

    StatusData CurStatusData; /* 用于存放系统状态的数据结构 */

    std::vector<ServiceClient*> ClientsList;  /* 当前订阅系统状态的抽象客户端列表 */

};