#include "ser.h"

using namespace std;
void Lazer::transferinit(double yaw)
{
    double x1 = lazer_param.lazerx-init_param.firstx ;
    double y1 = lazer_param.lazery-init_param.firsty ;
    init_param.lazerx = cos(yaw) * x1 + sin(yaw) * y1;
    init_param.lazery = cos(yaw) * y1 - sin(yaw) * x1;
    //init_param.yaw = lazer_param.yaw;
    //printf("x1:%.3f,y1:%.3f\n",x1,y1);
    //printf("x%.3f\n",init_param.firstx);
    //printf("y%.3f\n",init_param.firsty);

}
void Lazer::tranferinitpoint(double x,double y,double yaw)
{
    double x1 = x-init_param.firstx ;
    double y1 = y-init_param.firsty ;
    x = cos(yaw) * x1 + sin(yaw) * y1;
    y = cos(yaw) * y1 - sin(yaw) * x1;

}
void Lazer::transfertw(double yaw)
{
    double x1 = m_lazer_param.lazerx-lazer_param.firstx ;
    double y1 = m_lazer_param.lazery-lazer_param.firsty ;
    init_param.lazerx = cos(yaw) * x1 + sin(yaw) * y1;
    init_param.lazery = cos(yaw) * y1 - sin(yaw) * x1; 

} 

void Lazer::transfer() {
    double x1 = m_lazer_param.lazerx-lazer_param.firstx ;
    double y1 = m_lazer_param.lazery-lazer_param.firsty ;
    lazer_param.lazerx = cos(m_lazer_param.yaw) * x1 + sin(m_lazer_param.yaw) * y1;
    lazer_param.lazery = cos(m_lazer_param.yaw) * y1 - sin(m_lazer_param.yaw) * x1;
    //lazer_param.lazerx = cos(lazer_param.firstyaw) * x1 + sin(lazer_param.firstyaw) * y1;
    //lazer_param.lazery = cos(lazer_param.firstyaw) * y1 - sin(lazer_param.firstyaw) * x1;
    lazer_param.yaw=m_lazer_param.yaw-lazer_param.firstyaw;
    //printf("x1:%.3f,y1:%.3f,cos:%.3f,sin:%.3f\n",x1,y1,cos(param.yaw-param.firstyaw),sin(param.yaw-param.firstyaw));
    //printf("xxx%.3f,xxx%.3f",lazer_param.lazerx,lazer_param.lazery);
}

// 里程计回调函数
void Lazer::odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    //ROS_INFO("123");
    latest_msg = msg;
}

// 定时器回调函数
void Lazer::timeCallback(const ros::TimerEvent&) {
    //ROS_INFO("32123");
    if (latest_msg) 
    {
        m_lazer_param.lazerx = (latest_msg->pose.pose.position.x*100);
        m_lazer_param.lazery = (latest_msg->pose.pose.position.y*100);
        geometry_msgs::Quaternion siyuanshu = latest_msg->pose.pose.orientation;
        double x = siyuanshu.x;
        double y = siyuanshu.y;
        double z = siyuanshu.z;
        double w = siyuanshu.w;
        m_lazer_param.yaw = atan2(2 * (x * y + w * z), 1 - 2 * (y * y + z * z));
        //printf("x: %f, y: %f, yaw: %f\n", m_lazer_param.lazerx, m_lazer_param.lazery, m_lazer_param.yaw * 180 / M_PI);
        //lazer_param.lazerx =.0f;
    }
}

void Lazer::getlazerinit()
{
    init_param.lazerx = lazer_param.lazerx;
    init_param.lazery = lazer_param.lazery;
    init_param.firstyaw = lazer_param.yaw;
}
void Lazer::getlazerparaminitfirst()
{
    init_param.firstx = lazer_param.lazerx;
    init_param.firsty = lazer_param.lazery;
    init_param.firstyaw = lazer_param.yaw;
    //printf("x:%.3f,y:%.3f",init_param.firstx,init_param.firsty);
}
void Lazer::getlazerparam()
{
    lazer_param.lazerx = m_lazer_param.lazerx;
    lazer_param.lazery = m_lazer_param.lazery;
    lazer_param.yaw = m_lazer_param.yaw-lazer_param.firstyaw;
    //printf("%.3f,%.3f\n",lazer_param.yaw,lazer_param.firstyaw);
    //printf("%g,%g\n",param.lazerx,param.lazery);
    transfer();
    //printf("%.3f,%.3f\n",lazer_param.lazerx,lazer_param.lazery);
}
void Lazer::uplazerfirst()
{

    lazer_param.firstx = m_lazer_param.lazerx;
    lazer_param.firsty = m_lazer_param.lazery;
    lazer_param.firstyaw = m_lazer_param.yaw;
    //printf("xx:%.3f,yy:%.3f",lazer_param.firstx,lazer_param.firsty);
    //printf("\nfirst%.3f,%.3f\n",m_lazer_param.lazerx,m_lazer_param.lazery);
}
void* Lazer::_Lazerinit(void *arg)
{
    LazerStart::lazer_params param=*(LazerStart::lazer_params*)arg;
    ros::init(param.argc, param.argv, "odometry_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/Odometry", 1000, &Lazer::odometryCallback,&param.lazer);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), &Lazer::timeCallback,&param.lazer);
    ros::spin();
    
}
void LazerStart::Start(int argc, char **argv,Lazer lazer)
{
    lazer_params lazers_params;
    lazers_params.argc=argc;
    lazers_params.argv=argv;
    lazers_params.lazer=lazer;
    thread_create(Lazer::_Lazerinit,&lazers_params,ml_threadDesc);
}

/*
int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "odometry_listener");
    ros::NodeHandle nh;
    ROS_INFO("~~~");
    // 订阅/Odometry话题
    ros::Subscriber sub = nh.subscribe("/Odometry", 1000, odometryCallback);

    // 设置定时器回调函数
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), timeCallback);

    // 保持ROS程序不退出，等待回调函数被调用
    ros::spin();

    return 0;
}
*/
