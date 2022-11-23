#pragma once

/* created by ysfyuan on 2022/10/6 */

#include <iostream>
#include <thread>
#include <vector>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <glog/logging.h>


/* 系统状态数据结构 */
struct StatusData{
    int StatusNum;
    int CurAcc;
    int CurSpeed;
    int CurPos;
    std::string CurTime;

};


class ServiceClient{
public:
    ServiceClient(int listen_fd, int service_type, const std::string& user_id, const std::string& password, const std::string& client_ipaddr);  /* 抽象客户端构造函数 */
    ~ServiceClient();  /* 析构函数 */

    void Subscribe();  /* 订阅系统状态 */

    void Publish(StatusData* new_status_data);    /* 信息推送函数 */

    std::string DataDump(StatusData* new_status_data);    /* 推送数据打包 */  

    void Control();   /* 客户端反向控制指令（列车控制相关延生） */

    void ClientStart();  /* 用户处理连接反向指令及人工心跳 */

    void Disconnect();  /* 用于断开连接，避免占用资源, 将状态码置为-1 */

    int GetClientFd() const;  /* 获取用于pair通信的fd */

    int GetClientStatus() const; /* 获取连接状态 */

private: 
    int ListenFd; /* 监听的文件描述符 */

    int ClientStatus; /* 抽象客户端的当前状态 */

    int ServiceType;  /* 客户申请的服务类型 */

    std::string  UserId;  /* 客户端账号 */

    std::string  Password;  /* 客户端密码 */

    std::string  Address;   /* 客户端的网络地址 */

    int ClientID; /* 抽象客户端在订阅类中客户列表的下标位置 */

};