/* Created by ysfyuan on 2022/10/10 */

#include "ServiceClient.h"


/* 客户类构造函数 */
ServiceClient::ServiceClient(int listen_fd, int service_type, const std::string& user_id, const std::string& password, const std::string& client_ipaddr)
    :   ListenFd(listen_fd), ServiceType(service_type), UserId(user_id), Password(password), Address(client_ipaddr)
{
    ClientStatus = 1;   /* 初始化，已连接状态 */
}


/* 析构函数 */
ServiceClient::~ServiceClient()
{
    /* 暂无需要释放的指针对象 */
}


/* 获取客户类的connfd */
int ServiceClient::GetClientFd() const
{
    return ListenFd;
}


/* 获取连接状态 */
int ServiceClient::GetClientStatus() const
{
    return ClientStatus;
}


/* 订阅系统状态信息 */
void ServiceClient::Subscribe()
{
    // for(int i = 0; i < )
}


/* TODO:用户处理连接反向指令及人工心跳 */
void ServiceClient::ClientStart()
{

} 


/* TODO:推送数据打包函数 */
std::string ServiceClient::DataDump(StatusData* new_status_data)
{
    
}


/* 系统状态推送函数 */
void ServiceClient::Publish(StatusData* new_status_data)
{
    std::cout << "Publish : " << std::endl;
    /* 将推送数据打包成协议格式 */
    std::string str = DataDump(new_status_data);

    std::cout << " Message size : " << str.size() << std::endl;
    std::cout << " Message : " << str << std::endl;

    /* tcp发送数据至客户端 */
    send(ListenFd, str.c_str(), str.size(), 0);

}

