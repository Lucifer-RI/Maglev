#include <iostream>
#include <unistd.h>
#include <time.h>

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
                /* TODO: 取出有效数据段 */
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
                /* TODO: 取出有效数据段 */
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
                /* TODO: 取出有效数据段 */
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


int main()
{
    time_t start_time;
    long long raw_time;
    // time(&start_time);
    // start_time = start_time + 8*3600;

    while(1)
    {
        time(&start_time);
        raw_time = start_time+8*3600;
        std::cout <<  raw_time << std::endl;
        std::cout <<  asctime(gmtime(&start_time)) << std::endl;
        usleep(1000000);
    }
    
    return 0;
}
