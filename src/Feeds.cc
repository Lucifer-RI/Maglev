/* Created by ysfyuan on 2022/10/8 */

#include <Feeds.h>

#define BUFFER_LEN 512 
#define MMAP_MAX_LEN 2048
#define CHR_DEV_MAG "/dev/ttyACM0"
#define CHR_DEV_IMU "/dev/ttyACM1"
#define CHR_DEV_RFID "/dev/ttyACM2"
#define MMAP_FILE_PATH "/home/jiang/data"

/* 构造函数 */
Feeds::Feeds(long long cur_time, int feeds_status)
    : CurTime(cur_time), FeedsStatus(feeds_status)
{
    /* 开启用于通信的共享文件 */
    mFdFile = open(MMAP_FILE_PATH, O_RDWR|O_CREAT);
    if(mFdFile<0)
    {
        std::cout<<"open file failed!"<<MMAP_FILE_PATH<<std::endl;
        exit(1);
    }
    /* 共享文件映射到硬盘中，获取文件内容首地址MmapAddr */
    mAddr = mmap(nullptr, MMAP_MAX_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, mFdFile, 0);
    if(mAddr == NULL)
    {
        std::cout<<"Mmap failed !"<<std::endl;
        exit(1);
    }
    else
    {
        /* 共享文件复位 */
        std::cout << "New Mmap File !" << std::endl;
        memset(mAddr, 0, MMAP_MAX_LEN);
    }
    std::cout << "Mmap_addr : " << mAddr << std::endl;
    mStartAddr = static_cast<RawData*>(mAddr+16);
    std::cout << "Start Address : "<< mStartAddr <<std::endl;
}


/* 析构函数，释放共享文件、文件描述符以及指针对象 */
Feeds::~Feeds()
{
    /* 关闭mmap文件 */
    munmap(mAddr, MMAP_MAX_LEN);
    close(mFdFile);
    close(mFdMag);
    close(mFdIMU);
    close(mFdRFID);
}


/* 获取共享文件读标志位 */
int Feeds::GetReadIndex()
{
    mReadIndex = *(static_cast<int*>(mAddr));
    return mReadIndex;
}


/* 更新读标志位 */
void Feeds::UpdateReadIndex()
{
    ++(*(static_cast<int*>(mAddr)));
    mReadIndex = *(static_cast<int*>(mAddr));
    return;
}


/* 获取共享文件写标志位 */
int Feeds::GetWriteIndex()
{
    mWriteIndex = *(static_cast<int*>(mAddr+8));
    return mWriteIndex;
}


/* 更新写标志位 */
void Feeds::UpdateWriteIndex()
{
    ++(*(static_cast<int*>(mAddr+8)));
    mWriteIndex = *(static_cast<int*>(mAddr+8));
    return;
}


/* 向共享文件中写入Rawdata */
void Feeds::WriteData(RawData* pData)
{
    std::cout << "Writing RawData !!! WriteIndex : " << mWriteIndex << std::endl;
    memcpy(mStartAddr+mWriteIndex, pData, sizeof(RawData));
    UpdateWriteIndex();
    return;

}


/* 数据源启动函数 */
int Feeds::FeedsOpen()
{   
    /* 三个串口数据缓冲区 */
    char BufferMagnet[BUFFER_LEN];
    char BufferIMU[BUFFER_LEN];
    char BufferRFID[BUFFER_LEN];

    mFdMag = open(CHR_DEV_MAG,O_RDWR | O_NOCTTY);
    if(mFdMag < 0)
    {
        std::cout<<"open device magnet failed!"<<CHR_DEV_MAG<<std::endl;
        return -1;
    }
    std::cout<<"FdMag is :"<<mFdMag<<std::endl;

    mFdIMU = open(CHR_DEV_IMU,O_RDWR | O_NOCTTY);
    if(mFdIMU < 0)
    {
        std::cout<<"open device IMU failed!"<<CHR_DEV_IMU<<std::endl;
        return -1;
    }
    std::cout<<"FdIMU is :"<<mFdIMU<<std::endl;
    
    mFdRFID = open(CHR_DEV_RFID,O_RDWR | O_NOCTTY);
    if(mFdRFID < 0)
    {
        std::cout<<"open device RFID failed!"<<CHR_DEV_IMU<<std::endl;
        return -1;
    }
    std::cout<<"FdRFID is :"<<mFdRFID<<std::endl;    

    /* 轮询的机制接收多个异步串口的缓冲区数据 */
    RawData Renew;  /* 临时结构体，用于数据刷新纪录 */
    memset(&Renew,0,sizeof(Renew)); 
    int LenMag, LenIMU, LenRFID;

    /* 采集时间 */
    time_t raw_time;

    while(1)
    {
        /* 此处的buffer数据不一定与名称对应，可以默认为三个相同缓冲区 */
        LenMag=read(mFdMag, BufferMagnet, sizeof(BufferMagnet));
        LenIMU=read(mFdIMU, BufferIMU, sizeof(BufferIMU));
        LenRFID=read(mFdRFID, BufferRFID, sizeof(BufferRFID));

        /* 可在此处限制系统采样频率,此处为0.5s一次 */
        usleep(500000);

        /* 用于存放原始数据协议解析值 */
        std::pair<int,int> ret_parser;

        while(LenMag > 0 || LenIMU > 0 || LenRFID > 0)
        {   
            if(LenMag > 0)
            {
                ret_parser = RenewParser(BufferMagnet, LenMag);
                switch(ret_parser.first)
                {
                    case 1:
                        Renew.Magnet = ret_parser.second; 
                        Renew.MagnetFlag = 1; 
                        break;
                    case 2:
                        Renew.IMU = ret_parser.second;
                        Renew.IMUFlag = 1;
                        break;
                    case 3:
                        Renew.RFIDpos = ret_parser.second;
                        Renew.RFIDFlag = 1;
                        break;
                    default: 
                        break;
                }
            }

            if(LenIMU > 0)
            {
                ret_parser = RenewParser(BufferIMU, LenIMU);
                switch(ret_parser.first)
                {
                    case 1:
                        Renew.Magnet = ret_parser.second; 
                        Renew.MagnetFlag = 1; 
                        break;
                    case 2:
                        Renew.IMU = ret_parser.second;
                        Renew.IMUFlag = 1;
                        break;
                    case 3:
                        Renew.RFIDpos = ret_parser.second;
                        Renew.RFIDFlag = 1;
                        break;
                    default: 
                        break;
                }
            }
    
            if(LenRFID > 0)
            {
                ret_parser = RenewParser(BufferRFID, LenRFID);
                switch(ret_parser.first)
                {
                    case 1:
                        Renew.Magnet = ret_parser.second; 
                        Renew.MagnetFlag = 1; 
                        break;
                    case 2:
                        Renew.IMU = ret_parser.second;
                        Renew.IMUFlag = 1;
                        break;
                    case 3:
                        Renew.RFIDpos = ret_parser.second;
                        Renew.RFIDFlag = 1;
                        break;
                    default: 
                        break;
                }
            }
        }
        
        /* 完成临时结构体的更新则清空缓冲 */
        memset(BufferMagnet,0,sizeof(BufferMagnet));
        memset(BufferIMU,0,sizeof(BufferIMU));
        memset(BufferRFID,0,sizeof(BufferRFID));

        /* 时间戳设置 */
        time(&raw_time);
        Renew.RawTime = raw_time;   /* long int to long long */
        std::cout << "TimeStamp : " << raw_time << std::endl; 
        
        /* 将临时结构体内容拷贝到mmap文件中 */
        WriteData(&Renew);
    }
}


/* 串口数据协议解析 */
/* 函数功能： 可以处理三个通道中的任意一种传感器数据*/
/* 传入参数为 待定缓冲区的数据 */
/* 
    返回值： 无效数据则返回-100，
    数据为Magnet类型则返回1；
    数据为IMU类型则返回2；
    数据为RFID类型则返回3；
*/

/* TODO: 解析函数功能测试 */
std::pair<int,int> Feeds::RenewParser(char Buf[], int Length)
{
    std::pair<int,int> parser_result;
    int end_index = Length;
    int start_index;
    std::string real_data;
    while(end_index > 0 && Buf[end_index] != '}' && Buf[end_index] != ']' && Buf[end_index] != ')')
    {
        --end_index;
    }
    if(end_index <= 0)
    {
        parser_result.first = -100;
        return parser_result;
    }
    else
    {
        if(Buf[end_index] == '}')
        {
            start_index = end_index-1;
            while(start_index >= 0 && Buf[start_index] != '{')
            {
                --start_index;
            }
            if(start_index < 0)
            {
                parser_result.first = -100;
                return parser_result;
            }
            else
            {
                parser_result.first = 1;
                real_data.resize(end_index - start_index + 1);
                for(int i = start_index; i <= end_index; ++i)
                {
                    real_data[i-start_index] = Buf[i];
                }
                parser_result.second = MagParser(real_data);
            }
            
        }
        else if(Buf[end_index] == ']')
        {

            start_index = end_index-1;
            while(start_index >= 0 && Buf[start_index] != '[')
            {
                --start_index;
            }
            if(start_index < 0)
            {
                parser_result.first = -100;
                return parser_result;
            }
            else
            {
                parser_result.first = 2;
                real_data.resize(end_index - start_index + 1);
                for(int i = start_index; i <= end_index; ++i)
                {
                    real_data[i-start_index] = Buf[i];
                }
                parser_result.second = IMUParser(real_data);
            }
        }
        else if(Buf[end_index] == ')')
        { 
            start_index = end_index-1;
            while(start_index >= 0 && Buf[start_index] != '(')
            {
                --start_index;
            }
            if(start_index < 0)
            {
                parser_result.first = -100;
                return parser_result;
            }
            else
            {
                parser_result.first = 3;
                real_data.resize(end_index - start_index + 1);
                for(int i = start_index; i <= end_index; ++i)
                {
                    real_data[i-start_index] = Buf[i];
                }
                parser_result.second = RFIDParser(real_data);
            }
        }
        else
        {
            std::cout << "Error parsing ..." << std::endl;
        }
    }
    return  parser_result;
}


/* Magnet传感器数据解析 */
int Feeds::MagParser(std::string& MagData)
{

}


/* IMU传感器数据解析 */
int Feeds::IMUParser(std::string& IMUData)
{

}


/* RFID传感器数据解析 */
int Feeds::RFIDParser(std::string& RFIDData)
{

}