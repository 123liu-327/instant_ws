#ifndef FLOW_END_FOLLOW_H
#define FLOW_END_FOLLOW_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>//摄像头
#include <image_transport/image_transport.h>
#include <sensor_msgs/LaserScan.h>//雷达
#include <sensor_msgs/Imu.h>//位姿(IMU)
#include <std_msgs/String.h>//标准消息类型
#include <geometry_msgs/Twist.h>//速度消息
#include<mutex>//C++互斥锁
#include <tf/transform_datatypes.h> //tf树
#include <csignal>//用于信号处理
#include <atomic>//保证信号线程安全
#include <algorithm>
#include <chrono> //用于处理时间和计时功能
#include <deque>//用于图像的缓冲队列
#include <nav_msgs/Odometry.h>
//定义全局数据类型
#include <stdint.h>

typedef unsigned char       uint8;   //  8 bits 
typedef unsigned short int  uint16;  // 16 bits 
typedef unsigned long int   uint32;  // 32 bits 
// typedef unsigned long long  uint64;  // 64 bits 

typedef char                int8;    //  8 bits 
typedef short int           int16;   // 16 bits 
typedef long  int           int32;   // 32 bits 
// typedef long  long          int64;   // 64 bits 

//结果图像大小
#define RESULT_ROW 480
#define RESULT_COL 640
//透视图图像大小
#define USED_ROW   480
#define USED_COL   640

//图像处理数据结构
extern double change_un_Mat[3][3];
extern double invMat[3][3];
extern int point_map[RESULT_ROW][RESULT_COL][2];//存储对应点的坐标
extern uint8_t *PerImg_ip[RESULT_ROW][RESULT_COL];
extern uint8_t SimBinImage[RESULT_ROW][RESULT_COL];
#define PER_IMG    SimBinImage //SimBinImage:用于透 ? 变换的图像
#define ImageUsed  *PerImg_ip  //*PerImg_ip定义使用的图像，ImageUsed为用于巡线和识别的图 ?

//查找表数据结构
extern float mapx[RESULT_ROW][RESULT_COL];
extern float mapy[RESULT_ROW][RESULT_COL];

//速度发布订阅数据结构
extern ros::Publisher pub;
extern ros::Publisher end_pub;

//雷达回调数据结构
struct Laser
{
    float data;
    int index;
};
extern float slope;//记录板子斜率
extern float angle_deg;//记录斜率对应的角度
extern double current_yaw_lidar ;//将斜率对应的角度拷贝给此结构作为最初的角度偏差
extern bool is_lidar_update ;//判断雷达距离是否触发避障条件
#define LASE_MIN 0.35 //触发雷达避障的最小值

//摄像头回调函数数据结构
extern std::mutex frame_mutex;
extern cv::Mat frame;//图像

//IMU回调函数的数据结构
extern double current_yaw;//记录当前位姿信息
extern double curent_wz;//记录当前z轴角速度信息

//巡线类型数据结构
enum track_type_e {
    TRACK_LEFT,
    TRACK_MIDDLE,
    TRACK_RIGHT,
    
};
extern enum track_type_e track_type;

//起始消息，判断是否接收到红绿灯消息
extern bool run_car;
//信号处理
extern std::atomic<bool> sig_INT;
//直线避障Laser_linear的数据结构
extern ros::Time move_start_time;             // 记录动作开始时间
extern ros::Time move_start_time_after_laser; // 停车时间
extern int check ; //避障状态记录
extern float Dist_1 ;//记录雷达检测的最小距离

extern double pre_yaw ;
extern double pre_angle_deg;
extern bool check_after_laser ;
extern bool check_imu ;

//避障之后改变处理图像的范围
extern int after_bizhang_x;
extern int after_bizhang_y;



//图像处理的数据结构
extern uint8_t test_img[RESULT_ROW][RESULT_COL];//存储有图像转换的二维数组
//图像处理宏定义
inline int clip(int x, int low, int up)
{
    return x > up ? up : x < low ? low
                                 : x;
}
#define AT(img, x, y) ((img)->data[(y) * (img)->step + (x)])
#define AT_CLIP(img, x, y) AT(img, clip((x), 0, (img)->width - 1), clip((y), 0, (img)->height - 1))

//迷宫图像数据结构
typedef struct image {
    uint8_t *data;
    uint32_t width;
    uint32_t height;
    uint32_t step;
} image_t;

//迷宫方向
extern const int dir_front[4][2];
extern const int dir_frontleft[4][2];
extern const int dir_frontright[4][2];

//process_iamge中的数据结构
#define DEF_IMAGE(ptr, w, h)         {.data=ptr, .width=w, .height=h, .step=w}
extern image_t img_raw;//输入的原始图像
extern float begin_x ;//起始处理点
extern float begin_y;
#define POINTS_MAX_LEN  (300)//像素点最大值设置
extern int ipts0[POINTS_MAX_LEN][2];//左车道线像素点坐标数组
extern int ipts1[POINTS_MAX_LEN][2];//右车道线像素点坐标数组
extern int ipts0_num, ipts1_num;//左，右车导线像素点数量
extern float thres ; //局部梯度阈值判断
extern float block_size ;//像素处理的块大小
extern float clip_value ;//像素裁剪的阈值
extern float line_blur_kernel ;//模糊处理的核大小
extern float pixel_per_meter ;//像素到米的换算比例
//映射到真实世界坐标系（鸟瞰图)
extern float rpts0[POINTS_MAX_LEN][2];//左巡线映射点
extern float rpts1[POINTS_MAX_LEN][2];//右巡线映射点
extern int rpts0_num, rpts1_num;//左，右巡线映射点坐标
//模糊处理结果
extern float rpts0b[POINTS_MAX_LEN][2];//左线平滑后的点
extern float rpts1b[POINTS_MAX_LEN][2];//右线平滑后的点
extern int rpts0b_num, rpts1b_num;//左，右平滑后点的数量
//等距采样处理结果
extern float rpts0s[POINTS_MAX_LEN][2];//左线等距采样
extern float rpts1s[POINTS_MAX_LEN][2];//右线等距采样
extern int rpts0s_num, rpts1s_num;//左，右线等距采样点的数量
extern float sample_dist ;//等距采样的距离（单位：米）
//局部角度估计结果
extern float rpts0a[POINTS_MAX_LEN];//左线角度特征点
extern float rpts1a[POINTS_MAX_LEN];//右线角度特征点
extern int rpts0a_num, rpts1a_num;//局部角度估计点的数量
extern float angle_dist ;//角度特征提取距离间隔
//局部角度非极大值抑制结果
extern float rpts0an[POINTS_MAX_LEN];//左线角度点
extern float rpts1an[POINTS_MAX_LEN];//右线角度点
extern int rpts0an_num, rpts1an_num;//对应数量
//巡线跟踪结果
extern float rptsc0[POINTS_MAX_LEN][2];//左线跟踪结果
extern float rptsc1[POINTS_MAX_LEN][2];//右线跟踪结果
extern int rptsc0_num, rptsc1_num;//点的数量
#define ROAD_WIDTH      (0.36)//道路宽度（单位：米）
//重新等距采样结果
extern float rptsc0e[POINTS_MAX_LEN][2];//左线最终用于控制的结果
extern float rptsc1e[POINTS_MAX_LEN][2];//右线最终用于控制的结果
extern int rptsc0e_num, rptsc1e_num; //对应数量

//follow_line中的数据结构
extern int Ypt0_rpts0s_id, Ypt1_rpts1s_id;//Y型角点对应的序列下标
extern bool Ypt0_found, Ypt1_found;//Y型角点对应的序列下标
extern int Lpt0_rpts0s_id, Lpt1_rpts1s_id;//L型角点对应的序列下标
extern bool Lpt0_found, Lpt1_found;//L型角点对应的序列下标
extern const float PI;//Π的定义
extern bool is_straight0, is_straight1;//当前是否是直线车道段
extern float (*rpts)[2];//当前用于控制轨迹点指针
extern int rpts_num;//当前轨迹点数量
extern int zeroCount ;//连续帧无法获取车道线次数
extern bool zero_flag; //多帧丢线时设为true
extern uint8_t img_line_data[RESULT_ROW][RESULT_COL];//可视化图像数组
extern image_t img_line;//存放img_line_data的图像结构
#define AT_IMAGE(img, x, y)          ((img)->data[(y)*(img)->step+(x)])//访问图像中像素值的宏定义

//雷达计算角度偏置
#define ANGLE_BIAS 2

extern ros::Time Global_move_timer;
#define GLOBAL_TIMER 4.0

//法线平移距离
extern float Dis_Bias_Left;
extern float Dis_Bias_Right;
//角速度获取
extern double current_linear_velocity_x;
//线速度获取
extern double current_angular_velocity_z;
extern ros::Time last_imu_time;  // 上一次IMU消息时间
extern bool imu_first_msg ;

extern bool check_L_0;
extern bool check_L_1;
extern bool local_corner_point ;

//里程计数据
extern float odom_dist;
extern bool is_start;

extern  double Time_local;

//环岛状态机
extern int Round_step;
//第一阶段斜率
extern float Round_step1_k;
//第一阶段对应角度
extern float angle_rad_step1;
extern float angle_deg_step1;
extern ros::Time Round_timer;
#define ROUND_R 0.45

extern float Laser_linear_dis;
extern bool Laser_dis_check ;
#undef ROS_INFO
#define ROS_INFO(...)  do {} while (0)
#endif
