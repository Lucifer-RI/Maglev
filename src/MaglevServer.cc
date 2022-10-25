/* Created by ysfyuan on 2022/10/8 */

#include "MaglevServer.h"


/* MaglevServer具体实现 */

/* 构造函数 */
MaglevServer::MaglevServer(int feeds_num, int services_num, int server_status, int fusions_num)
    : FeedsNum(feeds_num), ServicesNum(services_num), ServerStatus(server_status), FusionsNum(fusions_num)
{
    pCurFeed = new Feeds(0, 0);   
    /* Feeds参数1为cur_time,构造时置0, 参数2为feed_status,初始未开始工作置0 */
    pSysStatus = new SystemStatus(3);  
    /* SystemStatus参数1为status_num */

    /* Service列表和Fusion列表在开启后再将实例化指针添加到列表中 */

    /* 系统状态在初始化时为0，运行时为1，未初始化状态为-1*/
    ServerStatus = 0; 
}

/* 析构函数 */
MaglevServer::~MaglevServer()
{
    ServerStatus = -1;  /* 系统状态回到未初始化状态 */
    
    /* 析构函数中释放Server中的指针成员，来释放内存空间 */
    delete pCurFeed;
    delete pSysStatus;
    delete pFusion;

    for(int i = 0; i < ServicesList.size(); ++i)
    {
        if(ServicesList[i] != nullptr)
        {
            delete ServicesList[i];
        }
    }
}


/* 系统启动 */
void MaglevServer::ServerStart(const std::vector<int>& ServicesPort, const std::vector<std::string>& ServicesName, const std::string& FusionName)
{
    /* 开启Feeds 和 Service*/
    FeedsStart();
    if(ServicesPort.size() != ServicesName.size())
    {
        /* 服务端参数有误，端口数和服务数不匹配 */
        std::cout << "ServicePosts can't match Services" << std::endl;
    }

    for(int i = 0; i < ServicesPort.size(); ++i)
    {
        /* 遍历列表生成Service */
        /* Service构造的第二个参数为该服务的最大支持连接数 */
        ServicesList.push_back(new Service(0, ServicesPort[i], 100, ServicesName[i]));
    }
    
    /* 初始化数据融合类(基类) */
    pFusion = nullptr;  /* 在Fusion_startRun开始时再指向具体派生类 */

    /* 初始化系统状态类 */
    pSysStatus = new SystemStatus(3);

    /* 打印系统初始状态 */
    pSysStatus->StatusMonitor();
    
    /* 开启服务 */
    ServiceStart();

}


void MaglevServer::FeedsStart()
{
    if(pCurFeed->FeedsOpen() < 0)
    {
        /* 具体启动失败可根据返回值扩展区分 */
        std::cout << "Feeds open failed" << std::endl;
    }
}


/* FusionStart */
// void MaglevServer::FusionStart(DataFusion* pExactlyFusion)
// {
//     pFusion = pExactlyFusion;
//     pFusion->RunFunc();
// }


void MaglevServer::ServiceStart()
{
    for(int i = 0; i < ServicesList.size(); ++i)
    {
        /* 支持多种服务类型，目前为数据采集事件驱动发送 */
        ServicesList[i]->ServiceOpen();   /* 每个Loop都是一个EpollIO线程 */
    }
}

void MaglevServer::ServerEnd()
{
    /* TODO：用于数据落地后处理等 */
}


