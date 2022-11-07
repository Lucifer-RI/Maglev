#include <iostream>
#include <unistd.h>
#include <time.h>

int main()
{
    time_t start_time;
    // time(&start_time);
    // start_time = start_time + 8*3600;

    while(1)
    {
        time(&start_time);
        std::cout <<  start_time << std::endl;
        usleep(500000);
    }
    
    return 0;
}
