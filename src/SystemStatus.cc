/* Created by ysfyuan on 2022/10/13 */

#include "SystemStatus.h"


/* 代码运行即创建单例 */
SystemStatus* SystemStatus::StatusInstance = new (std::nothrow) SystemStatus(3);

/* 获取单例 */
SystemStatus* SystemStatus::GetInstance()
{
    if(StatusInstance == nullptr)
    {
        StatusInstance = new (std::nothrow) SystemStatus(3);
    }
    return StatusInstance;
}


/* 构造函数 */
/* 此处注意： 默认参数只需要在头文件中定义一次，源文件中声明即可，否则触发重复定义 */
SystemStatus::SystemStatus(int status_num)
    :  StatusNum(status_num)
{
    /* 清空缓存系统状态数据 */
    CurStatusData = {};
    memset(&CurStatusData, 0, sizeof(CurStatusData));
    ClientsList = {};  /* 初始化订阅列表为空 */
}


/* 析构函数 */
SystemStatus::~SystemStatus()
{
    for(int i = 0; i < ClientsList.size(); ++i)
    {
        /* 若ClientList[i] 不为空指针，则释放所指内存空间 */
        if(ClientsList[i])
        {
            delete ClientsList[i];
        }
    }
}


/* 添加客户订阅系统状态 */
int SystemStatus::AddClient(ServiceClient* new_client)
{
    for(int i = 0; i < ClientsList.size(); ++i)
    {
        if(ClientsList[i] == nullptr)
        {
            ClientsList[i] = new_client;
            /* 返回该客户在订阅列表中的下标 */
            return i;  
        }
    }
    /* 在列表中无空位，则在表尾添加 */
    ClientsList.emplace_back(new_client);
    return ClientsList.size() - 1;
}


/* 删除已订阅用户 */
int SystemStatus::DelClient(ServiceClient* del_client)
{
    /* 找到待删除客户的迭代器 */
    auto it = find(ClientsList.begin(), ClientsList.end(), del_client);
    if(it != ClientsList.end())
    {
        /* 将该指针指向空 */
        ClientsList[it - ClientsList.begin()] = nullptr;
        return 1;  /* 返回1则说明该用户已被成功删除 */
    }
    return 0;   /* 返回0则说明该用户未订阅 */
}


/* 向订阅用户推送信息 */
void SystemStatus::NotifyClients()
{
    for(int i = 0; i < ClientsList.size(); ++i)
    {
        /* 在这里完成事件驱动的传递 */
        if(ClientsList[i])
        {
            /* 若客户端主动断开连接，则在此处进行服务端的断连 */
            /* 客户状态码为0，则对方已断开 */
            /* 客户状态码为1，则通信正常 */
            if(ClientsList[i]->GetClientStatus() == 0)
            {
                ClientsList[i]->Disconnect();
                delete ClientsList[i];
                ClientsList[i] = nullptr;
            }
            else
            {
                std::cout << "Notify New System Status: " << std::endl;
                ClientsList[i]->Publish(&CurStatusData);
            }
        }
    }
}


/* 更新系统状态数据, 供上流模块使用的更新接口 */
void SystemStatus::UpdateStatus(StatusData* pStatus)
{
    /* 更新前先清空 */
    memset(&CurStatusData, 0, sizeof(CurStatusData));
    /* 指针赋值 */
    CurStatusData = static_cast<StatusData>(*pStatus);

    /* 更新至客户端 */
    this->NotifyClients();
    /* 打印输出，用于开发人员调试 */
    this->StatusMonitor();
    return;
}



void SystemStatus::StatusMonitor()
{
    /* 打印用于开发过程验证 */
    std::cout << "Got Update ("  << CurStatusData.CurTime << "): " << " Acc -> " <<CurStatusData.CurAcc << " m/s2 " << " Speed -> " << (double)CurStatusData.CurSpeed /10000 << " m/s "
    << " Pos -> " << (double)CurStatusData.CurPos / 10000 << " m"<< std::endl;
}
 

