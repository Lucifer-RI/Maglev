/* Created by ysfyuan on 2022/10/8 */

#include <Feeds.h>

#define BUFFER_LEN 128 
#define MMAP_MAX_LEN 2048*2048
#define CHR_DEV_MAG "/dev/ttyACM0"
#define CHR_DEV_IMU "/dev/ttyUSB0"
#define CHR_DEV_RFID "/dev/ttyUSB1"
#define MMAP_FILE_PATH "/home/ysfyuan/maglev_mmap_data"

/* 串口初始化配置 */
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop);

/* 构造函数 */
Feeds::Feeds(long long cur_time, int feeds_status)
    : CurTime(cur_time), FeedsStatus(feeds_status)
{
    /* 开启用于通信的共享文件 */
    mFdFile = open(MMAP_FILE_PATH, O_RDWR|O_CREAT, 0777);
    // std::cout << " MMap FD :  " << mFdFile << std::endl;
    if(mFdFile < 0)
    {
        std::cout<<"open file failed! Path : "<<MMAP_FILE_PATH<<std::endl;
        exit(1);
    }
    /* Mmap文件地址使用前应设置虚拟文件对应大小 */
    ftruncate(mFdFile, MMAP_MAX_LEN);
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
    mStartAddr = static_cast<RawData*>(mAddr+2*sizeof(int));
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
    std::cout << "Cur ReadIndex : " << mReadIndex << std::endl;
    return;
}


/* 获取共享文件写标志位 */
int Feeds::GetWriteIndex()
{
    mWriteIndex = *(static_cast<int*>(mAddr+sizeof(int)));
    return mWriteIndex;
}


/* 更新写标志位 */
void Feeds::UpdateWriteIndex()
{
    ++(*(static_cast<int*>(mAddr+sizeof(int))));
    mWriteIndex = *(static_cast<int*>(mAddr+sizeof(int)));
    return;
}


/* 向共享文件中写入Rawdata */
void Feeds::WriteData(RawData* pData)
{
    std::cout << "Writing RawData !!! WriteIndex : " << mWriteIndex << std::endl;
    //std::cout << pData->Magnet <<" -> " << pData->MagnetFlag <<std::endl;
    memcpy(mStartAddr+mWriteIndex, pData, sizeof(RawData));

    //std::cout << (mStartAddr+mWriteIndex)->RawTime << std::endl;
    UpdateWriteIndex();
    return;
}


/* 从共享文件中读取最新数据 */
/* 读取的数据为迭代过的数据则返回0，未迭代过的数据则返回1 */
int Feeds::ReadData(RawData* pDataOut)
{
    if(mWriteIndex == 0 || mReadIndex > mWriteIndex)
    {
        /* 此时为系统初始化，还没有任何采集数据 */
        std::cout << "Get nothing from Feeds !!!, Wait a moment..." << std::endl;
        return -1;
    }
    if(mReadIndex == mWriteIndex)
    {
        std::cout << "Wait Data from Feeds !!! Got Lastest Data !!!" << std::endl;
        /* 此时情况为新数据都已经读取完毕，没有可用的新数据，则用最近一次的数据进行迭代 */
        /* 此处获取的数据为写标志位前一位，即最新数据 */
        // if((mStartAddr+mWriteIndex-1)->RFIDFlag == 1)
        // {
        //     /* 在mmap文件中标注出以及使用过的RFID数据 */
        //     (mStartAddr+mWriteIndex-1)->RFIDFlag = 2;
        // }
        memcpy(pDataOut, mStartAddr+mReadIndex-1, sizeof(RawData));
        return 0;
    }
    std::cout << "Reading Data !!! ReadIndex : " << mReadIndex << std::endl;
    memcpy(pDataOut, mStartAddr+mReadIndex, sizeof(RawData));
    // std::cout << pDataOut->Magnet <<" -> " << pDataOut->MagnetFlag <<std::endl;
    if((mStartAddr+mReadIndex)->RFIDFlag == 1)
    {
        std::cout << "/********************Correction******************************/" << std::endl; 
        (mStartAddr+mReadIndex)->RFIDFlag = 2;
    }
    // std::cout << "Read RFID flag : "<<pDataOut->RFIDFlag << std::endl;
    UpdateReadIndex();
    return 1;
}


/* 开启Feeds线程 */
int Feeds::FeedsOpen()
{
    /* 开启线程以及导入Loop函数 */
    std::thread feeds_thread(&Feeds::FeedsLoop, this);
    /* 主线程继续初始化其他模块 */
    feeds_thread.detach();
    return 0; 
}


/* 数据源启动函数 */
void Feeds::FeedsLoop()
{   
    std::cout << "Feeds Thread Open! " << std::endl;
    /* 三个串口数据缓冲区 */
    char BufferMagnet[BUFFER_LEN];
    char BufferIMU[BUFFER_LEN];
    char BufferRFID[BUFFER_LEN];

    /* 开启驱动文件 */
    mFdMag = open(CHR_DEV_MAG, O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY , 0777);
    if(mFdMag < 0)
    {
        std::cout<<"open device magnet failed! "<<CHR_DEV_MAG<<std::endl;
        // return -1;
    }
    else
    {
        set_opt(mFdMag, 9600, 8, 'N', 1); 
    }
    std::cout<<"FdMag is :"<<mFdMag<<std::endl;

    /* 开启驱动文件 */
    mFdIMU = open(CHR_DEV_IMU,O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY , 0777);
    if(mFdIMU < 0)
    {
        std::cout<<"open device IMU failed!"<<CHR_DEV_IMU<<std::endl;
    }
    else
    {
        set_opt(mFdIMU, 115200, 8, 'N', 1); 
    }
    std::cout<<"FdIMU is :"<<mFdIMU<<std::endl;
    
    /* 开启驱动文件 */
    mFdRFID = open(CHR_DEV_RFID,O_RDWR | O_NOCTTY | O_NONBLOCK | O_NDELAY, 0777);
    if(mFdRFID < 0)
    {
        std::cout<<"open device RFID failed!"<<CHR_DEV_RFID<<std::endl;
    }
    else
    {
        //驱动文件映射成功后，进行串口初始化
        set_opt(mFdRFID, 115200, 8, 'N', 1);    
    }
    std::cout<<"FdRFID is :"<<mFdRFID<<std::endl;    

    /* 轮询的机制接收多个异步串口的缓冲区数据 */
    RawData Renew;  /* 临时结构体，用于数据刷新纪录 */
    memset(&Renew,0,sizeof(Renew)); 
    int LenMag = 0;
    int LenIMU = 0;
    int LenRFID = 0;

    /* 采集时间 */
    time_t raw_time;
    
    /* 有效数据标志位 */
    int valid_flag = 0;

    while(1)
    {
        /* 此处的buffer数据不一定与名称对应，可以默认为三个相同缓冲区 */
        if(mFdMag != -1)
        {
            LenMag=read(mFdMag, BufferMagnet, sizeof(BufferMagnet));
            std::cout<< "Lenmag: "<< LenMag  << std::endl;
            // << "  ValueMag: "<< BufferMagnet << std::endl;
        }
        if(mFdIMU != -1)
        {
            LenIMU=read(mFdIMU, BufferIMU, sizeof(BufferIMU));
            std::cout<< "LenIMU: "<< LenIMU  << std::endl;
            // <<" ValueIMU: " << BufferIMU << std::endl;
        }
        if(mFdRFID != -1)
        {
            LenRFID=read(mFdRFID, BufferRFID, sizeof(BufferRFID));
            std::cout << "LendRFID: " << LenRFID << " -> " << BufferRFID << std::endl;
        }

        /* TODO: 此处设置仿真数据源 */
        /* 仿真Magnet数据 */
        
        /* 可在此处限制系统采样频率,此处为0.05s一次 */
        usleep(200000);

        /* 用于存放原始数据协议解析值 */
        std::pair<int,int> ret_parser;
        /* 有效标志位先置0 */
        valid_flag = 0;

        while(LenMag > 0 || LenIMU > 0 || LenRFID > 0)
        {
            if(LenMag > 0)
            {
                ret_parser = RenewParser(BufferMagnet, LenMag);
                /* 如果不为无效数据，则置为有效标志位为1 */
                if(ret_parser.first != -100)
                {
                    valid_flag = 1;
                }
                if(ret_parser.first == 1)
                {
                    Renew.Magnet = ret_parser.second; 
                    Renew.MagnetFlag = 1; 
                    // std::cout << "MagData!!! " << Renew.Magnet ; 
                }
                LenMag = -1;
            }

            if(LenIMU > 0)
            {
                ret_parser = RenewParser(BufferIMU, LenIMU);
                /* 如果不为无效数据，则置为有效标志位为1 */
                if(ret_parser.first != -100)
                {
                    valid_flag = 1;
                }
                if(ret_parser.first == 2)
                {
                    Renew.IMU = ret_parser.second;
                    Renew.IMUFlag = 1;
                }
                LenIMU = -1;
            }

            if(LenRFID > 0)
            {
                ret_parser = RenewParser(BufferRFID, LenRFID);
                /* 如果不为无效数据，则置为有效标志位为1 */
                if(ret_parser.first != -100)
                {
                    valid_flag = 1;
                }
                if(ret_parser.first == 3)
                {
                    Renew.RFIDpos = ret_parser.second;
                    Renew.RFIDFlag = 1;  
                }
                LenRFID = -1;
            }
        }

        /* 完成临时结构体的更新则清空缓冲 */
        memset(BufferMagnet,0,sizeof(BufferMagnet));
        memset(BufferIMU,0,sizeof(BufferIMU));
        memset(BufferRFID,0,sizeof(BufferRFID));

        /* 有效标志位为0，则此次采集数据无效，则不写入mmap中 */
        if(valid_flag == 0)
        {
            continue;
        }

        /* 时间戳设置 */
        time(&raw_time);
        Renew.RawTime = raw_time;   /* long int to long long */
        std::cout << "TimeStamp : " << raw_time << std::endl; 
        
        /* 将临时结构体内容拷贝到mmap文件中 */
        WriteData(&Renew);
        memset(&Renew, 0, sizeof(RawData));
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

std::pair<int,int> Feeds::RenewParser(char Buf[], int Length)
{
    std::pair<int,int> parser_result;
    int end_index = Length - 1;
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
                // std::cout  << "IMU parser : "<< parser_result.second << std::endl;
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
    int MagnetValue = 0;
    bool Valid = true;
    int flag = 1;
    int start = 1;
    if(MagData[1] == '-')
    {
        flag = -1;
        start = 2;
    }
    for (size_t i = start; i < MagData.size()-1; ++i)
    {
        if(isdigit(MagData[i]) == 0)
        {
            Valid = false;
            return -1;
        }
        else
        {
            MagnetValue = MagnetValue * 10 + (MagData[i] - '0') ;
        }
    }
    if(Valid)
    {
        return MagnetValue * flag;
    }
    return -1;
}


/* IMU传感器数据解析 */
int Feeds::IMUParser(std::string& IMUData)
{
    int IMUValue = 0;
    bool Valid = true;
    int flag = 1;
    int start = 1;
    if(IMUData[1] == '-')
    {
        flag = -1;
        start = 2;
    }
    for (size_t i = start; i < IMUData.size()-1; ++i)
    {
        if(isdigit(IMUData[i]) == 0)
        {
            Valid = false;
            return -1;
        }
        else
        {
            IMUValue = IMUValue * 10 + (IMUData[i] - '0') ;
        }
    }
    if(Valid)
    {
        // std::cout << "IMUVALUE :  " << IMUValue << std::endl;
        return (IMUValue / 10000) * flag;
    }
    return -1;
}


/* RFID传感器数据解析 */
int Feeds::RFIDParser(std::string& RFIDData)
{
    int RFIDValue = 0;
    bool Valid = true;
    for (size_t i = 1; i < RFIDData.size()-1; ++i)
    {
        if(isdigit(RFIDData[i]) == 0)
        {
            Valid = false;
            return -1;
        }
        else
        {
            RFIDValue = RFIDValue * 10 + (RFIDData[i] - '0') ;
        }
    }
    if(Valid)
    {
        return RFIDValue;
    }
    return -1;
}


/* 串口初始化配置 */
int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
	if  ( tcgetattr( fd,&oldtio)  !=  0) {  //检测串口是否可用
		perror("SetupSerial 1");
		return -1;
	}
	bzero( &newtio, sizeof( newtio ) );
	newtio.c_cflag  |=  CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	switch( nBits ) //设置数据位
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}

	switch( nEvent )//设置检验位
	{
	case 'O':
		newtio.c_cflag |= PARENB;
		newtio.c_cflag |= PARODD;
		newtio.c_iflag |= (INPCK | ISTRIP);
		break;
	case 'E': 
		newtio.c_iflag |= (INPCK | ISTRIP);
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		break;
	case 'N':  
		newtio.c_cflag &= ~PARENB;
		break;
	}

	switch( nSpeed ) //设置波特率
	{
		case 2400:
			cfsetispeed(&newtio, B2400);
			cfsetospeed(&newtio, B2400);
			break;
		case 4800:
			cfsetispeed(&newtio, B4800);
			cfsetospeed(&newtio, B4800);
			break;
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
		case 460800:
			cfsetispeed(&newtio, B460800);
			cfsetospeed(&newtio, B460800);
			break;
		default:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;
	}
	if( nStop == 1 )//设置停止位
		newtio.c_cflag &=  ~CSTOPB;
	else if ( nStop == 2 )
		newtio.c_cflag |=  CSTOPB;
		newtio.c_cc[VTIME]  = 0;
		newtio.c_cc[VMIN] = 0;
		tcflush(fd,TCIFLUSH);
	if((tcsetattr(fd,TCSANOW,&newtio))!=0) //设置串口参数
	{
		perror("com set error");
		return -1;
	}
	
	//	printf("set done!\n\r");
	return 0;
}