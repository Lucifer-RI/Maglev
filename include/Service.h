#pragma once 

/* created by ysfyuan on 2022/9/30 */

#include <iostream>
#include <vector>
#include <string>
#include <stdlib.h>
#include <thread>
#include <time.h>
#include <pthread.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/signal.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <string.h>
#include <ServiceClient.h>
#include <errno.h>



class Service{
public:
    Service(const int& service_type, const int& cur_listen_port, const int& max_connection_num, const std::string& service_name);  /* 构造函数 */
    
    ~Service();  /* 析构函数 */

    void ServiceOpen();    /* 开启监听线程 */

    void ServiceTreadLoop();  /* 主线程epoll架构，等待连接申请 */

    int DisconnectClient(int client_fd); /* 某个客户端断开连接,返回操作状态码 */  

    ServiceClient* AddServiceClient(int fd, int service_type, const std::string& user_id, const std::string& password, const std::string& client_ipaddr);   /* 添加该服务的客户，返回客户类指针 */

    void ClientThreadLoop(ServiceClient* client);  /* 开启线程与客户端进行pair通信 */ 

private:
    std::string ServiceName;

    int CurListenFd;   /* 当前申请连接的监听文件描述符 */

    int CurListenPort;    /* 开放的监听端口，客户端通过它来连接 */

    int MaxConnectionNum; /* 最大支持的连接数 */

    int ServiceType;  /* 服务类型 */

    int ServiceStatus = -1;   /* 服务当前状态 */

    std::vector<ServiceClient*>  ClientsList;  /* 服务端记录的连接客户列表 */

};