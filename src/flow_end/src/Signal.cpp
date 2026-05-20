#include <flow_end/follow.h>
#include <flow_end/Signal.h>
//自定义信号处理程序
void signalHandler(int signum)
{
    ROS_INFO("Receive Ctrl-C Process ending ...");
    sig_INT.store(true);
    exit(0);
    return ;
}
