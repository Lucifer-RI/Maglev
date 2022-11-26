/* Created by ysfyuan on 2022/10/20 */

#include "MaglevServer.h"


/* 
系统运行流程 ：
   ① 实例化MaglevServer
   ② 唯一实例化SystemStatus和数据融合抽象类DataFusion, 
   ③ 融合开始迭代, 始终取mmap共享文件中的最新数据 
   ④ 调用其中FeedStart、ServiceStart,服务端开始监听连接请求, 生成抽象客户类以及用户列表
   ⑤ Feed启动, 开始观测数据采集并存入mmap共享文件中, 实现数据融合估算以及信息推送 
*/

/* 系统工作流程实现 */
int main()
{
   /* 开启服务器 */
   MaglevServer *pServer = new MaglevServer(3, 1, 0, 1);
   /* 开启feed和service，feed和service都应该在单独线程中运行 */
   pServer->ServerStart({1024}, {"SpinPush"}, "KF");
   sleep(2);
   /* 初始化数据融合器 */
   DataFusion* pFusion = new KF(3, 3, 0, 3, "KF", pServer->GetCurFeeds());
   /* 主线程用于融合迭代计算 */
   pFusion->RunFunc();
   
   std::cout << "Server Close !" << std::endl;

   return 0;
}



