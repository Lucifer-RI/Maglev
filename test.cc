#include <iostream>
#include <unistd.h>
#include <string>
#include <stdlib.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <glog/logging.h>
#include <unistd.h>
#include <fcntl.h>
#include <memory.h>
#include <time.h>
#include <vector>  
#include <unordered_map>
#include <cmath> 

using namespace std;  

// #define PI 3.1415926
// float sinData[1024] = {0};

// std::pair<int,int> findPeaks(float *num,int count)  
// {   
//     // 峰值间距（磁轨长度）
//     uint16_t distance = 7000000; 
//     std::vector<int> signal(2,0);
//     uint16_t flag;
//     // 
//     vector<int> sign;  
//     for(int i = 1; i < count; i++)  
//     {  
//         float diff = num[i] - num[i-1];
//         //cout<<diff;  
//         if(diff>0)  
//         {  
//             if(!flag )
//             {
//                 signal[1]++;
//             }
//             sign.push_back(1);  
//         }  
//         else if(diff<0)  
//         { 
//             if(!flag )
//             {
//                 signal[0]++;
//             } 
//             sign.push_back(-1);  
//         }  
//         else  
//         {  
//             sign.push_back(0);  
//         }
//         if(signal[0] > 0 && signal[1] > 0 ) 
//         {
//             flag = 1;
//         }
//     } 

//     if(flag == 0)
//     {
//         return {-1,-1};
//     }

//     std::pair<int,int> ans;

//     vector<int> indMax;  
//     vector<int> indMin;  
    


//     for(int j = 1; j < sign.size(); j++)  
//     {   
//         int diff = sign[j]-sign[j-1]; 
//         if(diff > 0)  
//         { 
//             indMax.push_back(j);
//         }  
//         else if(diff < 0)  
//         {  
//             indMin.push_back(j);  
//         }  
//     }
      
//     /* TODO: 加上第一个波峰所在采样数据的位置信息，以及波峰波谷计算的相互验证 */
//     int posMax = indMax.size()*distance;
//     int posMin = indMin.size()*distance;
//     int posLast = indMax[0];
//     // ans.first = compare(posMax, posMin, posLast);


//     for(int m = 2;m<indMax.size();m++)     
//     {   
//         ans.second = (distance / (indMax[m]-indMax[m-2])) * 2;
//     }  
// }  


// void get_sin_tab( unsigned int point)
// {
//     int i = 0;
//     float hd = 0.0;         //弧度
//     float A = 1.0;        //峰值
//     float tem = 0;
//     for( i = 0; i < point; i++ )
//     {
//         tem = A*sin(2*PI*i/16);
     
//         sinData[i] = tem;
//     }
// }
 

// int MagParser(std::string &MagData)
// { 
//     int MagnetValue=0;
//     bool Valid= true;
//     for (size_t i = 1; i < MagData.size()-1; i++)
//     {
//         if(isdigit(MagData[i])==0)
//         {
//             Valid = false;
//             return -1;
//         }
//         else
//         {
//             MagnetValue = MagnetValue * 10 + MagData[i] - '0' ;
//         }
//     }
//     if(Valid)
//     {
//         return MagnetValue;
//     }
//     return -1;
// }



// /* 解析函数功能测试 */
// std::pair<int,int> RenewParser(char Buf[], int Length)
// {
//     std::pair<int,int> parser_result;
//     int end_index = Length-1;
//     int start_index;
//     std::string real_data;
//     while(end_index > 0 && Buf[end_index] != '}' && Buf[end_index] != ']' && Buf[end_index] != ')')
//     {
//         --end_index;
//     }
//     std::cout << end_index << std::endl; 
//     if(end_index <= 0)
//     {
//         parser_result.first = -100;
//         return parser_result;
//     }
//     else
//     {
//         if(Buf[end_index] == '}')
//         {
//             start_index = end_index-1;
//             while(start_index >= 0 && Buf[start_index] != '{')
//             {
//                 --start_index;
//             }
//             if(start_index < 0)
//             {
//                 parser_result.first = -100;
//                 std::cout <<  -100 << std::endl;
//                 return parser_result;
//             }
//             else
//             {
//                 parser_result.first = 1;
//                 real_data.resize(end_index - start_index + 1);
//                 for(int i = start_index; i <= end_index; ++i)
//                 {
//                     real_data[i-start_index] = Buf[i];
//                 }
//                 std::cout << real_data << std::endl;
//                 // parser_result.second = MagParser(real_data);
//             }
            
//         }
//         else if(Buf[end_index] == ']')
//         {

//             start_index = end_index-1;
//             while(start_index >= 0 && Buf[start_index] != '[')
//             {
//                 --start_index;
//             }
//             if(start_index < 0)
//             {
//                 parser_result.first = -100;
//                 std::cout <<  -100 << std::endl;
//                 return parser_result;
//             }
//             else
//             {
//                 parser_result.first = 2;
//                 real_data.resize(end_index - start_index + 1);
//                 for(int i = start_index; i <= end_index; ++i)
//                 {
//                     real_data[i-start_index] = Buf[i];
//                 }
//                 std::cout << real_data << std::endl;
//                 // parser_result.second = IMUParser(real_data);
//             }
//         }
//         else if(Buf[end_index] == ')')
//         { 
//             start_index = end_index-1;
//             while(start_index >= 0 && Buf[start_index] != '(')
//             {
//                 --start_index;
//             }
//             if(start_index < 0)
//             {
//                 parser_result.first = -100;
//                 std::cout <<  -100 << std::endl;
//                 return parser_result;
//             }
//             else
//             {
//                 parser_result.first = 3;
//                 real_data.resize(end_index - start_index + 1);
//                 for(int i = start_index; i <= end_index; ++i)
//                 {
//                     real_data[i-start_index] = Buf[i];
//                 }
//                 std::cout << real_data << std::endl;
//                 // parser_result.second = RFIDParser(real_data);
//             }
//         }
//         else
//         {
//             std::cout << "Error parsing ..." << std::endl;
//         }
//     }
//     std::cout << "Confident result-> " << parser_result.first  << " : " << real_data << std::endl;
//     return  parser_result;
// }

#define CHR_DEV_MAG "/dev/ttyACM0"

#define CHR_DEV_IMU "/dev/ttyUSB1"

#define MMAP_FILE_PATH "/home/ysfyuan/maglev_mmap_data"

#define MMAP_MAX_LEN 2048*2048

/* 数据源接收到的原生数据 */
struct RawData {
    int Magnet;  /* TMR磁场强度观测值 */
    int MagnetFlag; /* TMR传感器通道接收数据标识 */
    int MagMax;  /* 峰值标志位， 1为波峰，-1为波谷，0为非标征 */
    int IMU;    /* IMU传感器的方向加速度 */
    int IMUFlag;    /* IMU传感器通道接受数据标识 */
    int RFIDpos;    /* RFID传感器的定位观测值 */
    int RFIDFlag;   /* RFID传感器通道接受数据标识 */
    long long RawTime;    /* 记录每一次的数据采集驱动时间 */
    int Pos; /* Position   */
    int Speed; /* Speed */

};


int main()
{
    /* 开启驱动文件 */
    int mFdmmap = open(MMAP_FILE_PATH, O_RDWR, 0777);
    if(mFdmmap < 0)
    {
        std::cout<<"open MMAP failed! "<< CHR_DEV_IMU <<std::endl;
    }

    /* Mmap文件地址使用前应设置虚拟文件对应大小 */
    ftruncate(mFdmmap, MMAP_MAX_LEN);
    /* 共享文件映射到硬盘中，获取文件内容首地址MmapAddr */
    void* mAddr = mmap(nullptr, MMAP_MAX_LEN, PROT_READ|PROT_WRITE, MAP_SHARED, mFdmmap, 0);
    int WriteIndex = *(static_cast<int*>(mAddr + sizeof(int)));
    RawData* mData = static_cast<RawData*>(mAddr + 2*sizeof(int));
    std::cout << WriteIndex << std::endl; 
    for (int i = 0; i < WriteIndex; i++)
    {
        std::cout << " Magnetflag : " << (mData+i)->MagnetFlag << " Magnet : " << (mData+i)->Magnet << " RFIDflag : " <<(mData+i)->RFIDFlag << " RFIDpos : " << (mData+i)->RFIDpos <<std::endl;
    }

    return 0;  
}
