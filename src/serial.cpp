#include<stdio.h>
#include<string.h>
#include<sys/mman.h>
#include<fcntl.h>
#include<unistd.h>
#include <cstdlib>
#include<iostream>
#include<sys/wait.h>
#include <string>

#define LEN 1024 //259400
#define CHR_DEV_NAME "/dev/ttyACM1"
#define FILE_PATH "/home/jiang/data"
struct RawData  
{
    int Magnet; /* TMR磁场强度观测值 */
    int MagnetFlag; /* TMR传感器通道接收数据标识 */
    int IMUx;    /* IMU传感器的X方向加速度 */
    int IMUy;    /* IMU传感器的Y方向加速度 */
    int IMUz;    /* IMU传感器的Z方向加速度 */
    int IMUFlag;    /* IMU传感器通道接受数据标识 */
    int RFIDpos;    /* RFID传感器的定位观测值 */
    int RFIDFlag;   /* RFID传感器通道接受数据标识 */

    long long  RawTime;    /* 记录每一次的数据采集驱动时间 */
};
int pascal(char buf[])
{
	char *bufBegin = NULL;
	char *bufEnd = NULL;
	char *indexEnd =NULL;
	char *bufstart=NULL;
        int magnet =0;
        char magnetchar[8];
        int lenth=0;
        
	bufstart=buf;
	bufEnd = strrchr (buf, '}');
	lenth=bufEnd-bufstart;
	if(bufEnd == NULL)
	{
		printf("not found!\n");
	}
	else
	{ 
	indexEnd=bufEnd;
	
	while(*indexEnd!='{'&& lenth>= 0)
	{indexEnd--;
	lenth--;}
	
	if(*indexEnd!='{'){
	memset(buf,0,sizeof(buf));
	return -100;}
	memcpy(magnetchar,indexEnd+1,int(bufEnd-indexEnd));
	std::cout<<"memcp: "<<magnetchar<<std::endl;
	magnet=atoi(magnetchar);
	std::cout<<"atoi: "<<magnet<<std::endl;
	}
        memset(buf,0,sizeof(buf));
	return magnet;
}

int main()
{ int RxLen=0;
    int ret;
    int i = 0;
    int j = 0;
    RawData *initial;
    char buf[1024];
    //char *buf;
    //opne the device
    int fd = open(CHR_DEV_NAME,O_RDWR | O_NOCTTY);
     if(fd < 0)
    {
        printf("open device %s failed!\n",CHR_DEV_NAME);
        return -1;
    }
    printf("fd is %d\n",fd);

    //open the file
   int fd1 = open(FILE_PATH, O_RDWR|O_CREAT);

   
      if(fd1 < 0)
    {
        printf("open file %s failed!\n","/home/jiang/data");
        exit(1);
    }
    /*
    
    write(fd1,"1",1);
    lseek(fd1,LEN,SEEK_SET);
    pid_t pid = fork();
    if(pid == -1)
    {
        perror("fork error");
        close(fd);
        exit(1);
    }

    if(pid>0)
    {
    */
    
    void* mmp_addr = mmap(NULL,LEN,PROT_READ|PROT_WRITE,MAP_LOCKED|MAP_SHARED,fd1,0);
        if(mmp_addr == NULL){
            printf("mmap failed\n");
            return -1;}
    printf("mmp_addr is %p\n",mmp_addr);
    int index_read= *(static_cast<int*>(mmp_addr));
    printf("index_read is %d\n",index_read);
    int index_write=*(static_cast<int*>(mmp_addr+8));
    printf("index_write is %d\n",index_write);
    
    initial =static_cast<RawData*>(mmp_addr+16);
    //initial=mmp_addr+16;
    printf("initial is %p\n",initial);
    //struct
    RawData data1;
    while(1)
    {while(RxLen=read(fd, buf,sizeof(buf))>0)
    {//
    usleep(50000);

   std::cout<<"original data:"<<buf<<std::endl;
   int datajudge=pascal(buf);
  memset(buf,0,sizeof(buf));
   if(datajudge==-100)
   {continue;}
   memset(&data1,0,sizeof(data1));
   
    data1.Magnet=datajudge;
    
    RawData* temp_write=initial+index_write;
    memcpy(temp_write,&data1,sizeof(data1));
    index_write=index_write+1;
    }}
    close(fd);
    close(fd1);
    munmap(mmp_addr,LEN);
    return 0;
}
 
