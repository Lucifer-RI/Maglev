/* Created by ysfyuan on 2022/10/9 */

#include "Service.h"


#define gettid() syscall(__NR_gettid)
const int MAXlINE = 5;
const int OPEN_MAX = 100;
const int LISTEN_BACKLOG_LENGTH = 20;


/* 构造函数 */
Service::Service(const int& service_type, const int& cur_listen_port, const int& max_connection_num, const std::string& service_name)
    :ServiceType(service_type), CurListenPort(cur_listen_port), MaxConnectionNum(max_connection_num), ServiceName(service_name)
{
    /* 服务初始化状态为0，未初始化为-1 */
    ServiceStatus = 0;
}

/* 析构函数 */
Service::~Service()
{
    /* 订阅该服务的抽象客户列表进行内存释放 */
    for(int i = 0; i < ClientsList.size(); ++i)
    {
        delete ClientsList[i];
    }
    /* 状态置为-1 */
    ServiceStatus = -1;
}

/* 服务开启函数 */
void Service::ServiceOpen()
{
    /* 创建监听线程，实际线程函数在Loop中实现 */
    std::thread t(&Service::ServiceTreadLoop, this);
    std::cout << "Service open " << std::endl;
    /* 用detach，让主线程继续工作，不阻塞在此处 */
    t.detach();
}

void SignalHandler(int signum)
{
    std::cout << "Disconnect Signal !" << std::endl;
}

/* 服务监听loop */
void Service::ServiceTreadLoop()
{
    std::cout << "ThreadService tid : " << gettid() << std::endl;
    std::cout << "ThreadService pid : " << getpid() << std::endl;
    /* 此处进行信号处理是为了避免客户端主动断开后，服务端主动通信产生SIGPIPE信号触发crash */
    signal(SIGPIPE, SignalHandler);

    int sockfd, epfd;

    socklen_t ClientLen;

    struct epoll_event ev, events[20];  /* 定义感兴趣IO事件数组 */

    epfd = epoll_create(256);  /* 创建epoll句柄，用完后注意close */
    struct sockaddr_in ClientAddress;
    struct sockaddr_in ServiceAddress;

    /* 当前监听连接请求的文件描述符 */
    CurListenFd = socket(AF_INET, SOCK_STREAM, 0);
    std::cout << "CurListenFd: " << CurListenFd << std::endl;

    /* 构建感兴趣的epoll IO事件 ev */
    ev.data.fd = CurListenFd;    
    ev.events = EPOLLIN | EPOLLET;

    /* 将该事件节点添加到epoll监听红黑树上 */
    epoll_ctl(epfd, EPOLL_CTL_ADD, CurListenFd, &ev);
    /* 给服务地址先置0,再赋值 */
    memset(&ServiceAddress, 0, sizeof(ServiceAddress));
    ServiceAddress.sin_family = AF_INET;
    ServiceAddress.sin_addr.s_addr = htonl(INADDR_ANY);  /* 允许任何地址申请访问 */
    ServiceAddress.sin_port = htons(CurListenPort);   /* 设置监听端口 */

    /* while直到找到空闲port，并bind */
    while(bind(CurListenFd, (sockaddr*)&ServiceAddress, sizeof(ServiceAddress)) < 0)
    {
        /* 若不是端口占用的原因使得无法绑定，则打印errno并退出 */
        if(errno != EADDRINUSE)
        {
            std::cout << "Service bind failed: " << errno << std::endl;
            exit(-1);
        }
        std::cout << "Port already in used!, port : " << CurListenPort << std::endl;
        ServiceAddress.sin_port = htons(++CurListenPort);
        std::cout << "Retry bind port : " << CurListenPort << std::endl;
    }
    /* 打印监听端口 */
    std::cout << "Bind Success, Listen Port : " << CurListenPort << std::endl;
    /* 开始监听，listen中第二个参数为backlog队列的长度上限 */
    if(listen(CurListenFd, LISTEN_BACKLOG_LENGTH) < 0)
    {
        std::cout << "Listen Error !!" << std::endl; 
    }
    else
    {
        std::cout << "Listening : " << CurListenPort << std::endl;
    }

    int nfds;  /* 已就绪的感兴趣IO事件个数 */
    int connfd;  /* 从backlog队列中accept已就绪的连接申请，生成一个新的文件描述符 */
    std::string tempUserID, tempPassword;
    while(1)
    {
        /* epoll_wait 获取已就绪的感兴趣IO事件（在双向链表中，通过传入参数处获取） */
        /* events为侵入式的双向链表 */
        /* 第三个参数为感兴趣事件的最大大小，第四个参数为timeout */
        nfds = epoll_wait(epfd, events, 20, 500);
        for(int i = 0; i < nfds; ++i)
        {
            /* 遍历就绪事件数组，双向链表会被从内核中拷贝填入 */
            if(events[i].data.fd == CurListenFd)    /* 判断事件是否为连接申请 */
            {
                /* 建立连接（sock_stream,sock_packet）的文件描述符 */
                /* accept的第一个参数 ：sockfd  */
                /* 第二个参数 ： 客户端地址指针,accept会把获取到的客户端信息填入 */
                /* 第三个参数 ： 客户端信息长度 */
                connfd = accept(CurListenFd, (sockaddr*)&ClientAddress, &ClientLen);
                if(connfd < 0)
                {
                    std::cout << "accept error: " << connfd << std::endl;
                }
                char* client_ip = inet_ntoa(ClientAddress.sin_addr);
                std::cout << "Accpet a new connection from " << client_ip << ", connfd : " << connfd << std::endl;
                /* 建立连接后生成抽象客户类 */
                ServiceClient* new_service_client = AddServiceClient(connfd, ServiceType, tempUserID, tempPassword, client_ip);
                /* 客户类对系统状态进行订阅 */
                new_service_client->Subscribe();
                /* 在pair通信前，打印客户类的connfd信息 */
                std::cout << "Before PairTCP, ClientFd : " << new_service_client->GetClientFd() << std::endl;
                /* 开启线程进行pair通信, 注意类成员函数作为线程函数时的传参方式 */
                std::thread t_client(&Service::ClientThreadLoop, this, new_service_client);
                /* 主线程设置为非阻塞 */
                t_client.detach();
            }
        }
    }

}


/* 添加客户 */
ServiceClient* Service::AddServiceClient(int fd, int service_type, const std::string& user_id, const std::string& password, const std::string& client_ipaddr)
{
    /* 构造客户类，并插入服务类的客户列表中 */
    ServiceClient* new_service_client = new ServiceClient(fd, service_type, user_id, password, client_ipaddr);
    ClientsList.push_back(new_service_client);
    return new_service_client;
}


/* 客户类loop */
void Service::ClientThreadLoop(ServiceClient* client)
{
    std::cout << "Enter Client thread loop , connfd : " << client->GetClientFd() << std::endl;
    /* 开始与客户端的pair通信 */
    client->ClientStart();
    return;
}


/* 删除客户 */
int Service::DisconnectClient(int client_fd)
{
    int del_index = -1;
    for(int i = 0; i < ClientsList.size(); ++i)
    {
        if(ClientsList[i]->GetClientFd() == client_fd)
        {
            delete ClientsList[i];
            del_index = i;
        }
    }
    if(del_index >= 0)
    {
        ClientsList.erase(ClientsList.begin() + del_index);
        std::cout << "DisConnected from fd : " << client_fd << std::endl;
        return 0;
    }
    std::cout << "Can't find the client " << client_fd  << " from ClientList"<< std::endl;
    return -1;
}







