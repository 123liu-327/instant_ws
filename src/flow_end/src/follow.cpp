//自定义的有文件
#include <flow_end/follow.h>
#include <flow_end/ImagePerspectiveInit.h>
#include <flow_end/generateLookupTable.h>
#include <flow_end/Callback.h>
#include <flow_end/Signal.h>
#include <flow_end/follow_line.h>
#include <flow_end/PID.h>
#include "flow_end/follow.h"


// 这里是真正定义变量的地方（不能加 extern）
double change_un_Mat[3][3] = {{-2.897018, 2.446196, -388.368977},
                              {-0.061836, 1.194630, -756.140464},
                              {-0.000272, 0.008324, -4.335235}};
double invMat[3][3];
int point_map[RESULT_ROW][RESULT_COL][2];
uint8_t *PerImg_ip[RESULT_ROW][RESULT_COL];
uint8_t SimBinImage[RESULT_ROW][RESULT_COL];
float mapx[RESULT_ROW][RESULT_COL];
float mapy[RESULT_ROW][RESULT_COL];
ros::Publisher pub;
ros::Publisher end_pub;

float slope;
float angle_deg;
double current_yaw_lidar = 0;
bool is_lidar_update = false;

std::mutex frame_mutex;
cv::Mat frame;

double current_yaw = 0;
double curent_wz = 0;

enum track_type_e track_type = TRACK_RIGHT; // TRACK_RIGHT

bool run_car = false;
std::atomic<bool> sig_INT(false);

ros::Time move_start_time;
ros::Time move_start_time_after_laser;
int check = 0;
float Dist_1 =100.0;

double pre_yaw = 0;
double pre_angle_deg = 0;
bool check_after_laser = false;
bool check_imu = false;

int after_bizhang_x = 0;
int after_bizhang_y = 0;

uint8_t test_img[RESULT_ROW][RESULT_COL];

const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

image_t img_raw = DEF_IMAGE(NULL, RESULT_COL, RESULT_ROW);
float begin_x = 25;
float begin_y = 400;

int ipts0[POINTS_MAX_LEN][2];
int ipts1[POINTS_MAX_LEN][2];
int ipts0_num = 0, ipts1_num = 0;
float thres = 30;
float block_size = 7;
float clip_value = 1;
float line_blur_kernel = 7;
float pixel_per_meter = 500;
float rpts0[POINTS_MAX_LEN][2];
float rpts1[POINTS_MAX_LEN][2];
int rpts0_num = 0, rpts1_num = 0;
float rpts0b[POINTS_MAX_LEN][2];
float rpts1b[POINTS_MAX_LEN][2];
int rpts0b_num = 0, rpts1b_num = 0;
float rpts0s[POINTS_MAX_LEN][2];
float rpts1s[POINTS_MAX_LEN][2];
int rpts0s_num = 0, rpts1s_num = 0;
float sample_dist = 0.01;
float rpts0a[POINTS_MAX_LEN];
float rpts1a[POINTS_MAX_LEN];
int rpts0a_num = 0, rpts1a_num = 0;
float angle_dist = 0.1;
float rpts0an[POINTS_MAX_LEN];
float rpts1an[POINTS_MAX_LEN];
int rpts0an_num = 0, rpts1an_num = 0;
float rptsc0[POINTS_MAX_LEN][2];
float rptsc1[POINTS_MAX_LEN][2];
int rptsc0_num = 0, rptsc1_num = 0;
float rptsc0e[POINTS_MAX_LEN][2];
float rptsc1e[POINTS_MAX_LEN][2];
int rptsc0e_num = 0, rptsc1e_num = 0;

int Ypt0_rpts0s_id = 0, Ypt1_rpts1s_id = 0;
bool Ypt0_found = false, Ypt1_found = false;
int Lpt0_rpts0s_id = 0, Lpt1_rpts1s_id = 0;
bool Lpt0_found = false, Lpt1_found = false;
const float PI = 3.14159265358979323846f;
bool is_straight0 = false, is_straight1 = false;
float (*rpts)[2] = nullptr;
int rpts_num = 0;
int zeroCount = 0;
bool zero_flag = false;
uint8_t img_line_data[RESULT_ROW][RESULT_COL];
image_t img_line = DEF_IMAGE((uint8_t *)img_line_data, RESULT_COL, RESULT_ROW);
ros::Time Global_move_timer;
float Dis_Bias_Left = 0.0;//5.0
float Dis_Bias_Right = 0.0;//5.0
double  current_linear_velocity_x=0;
//线速度获取
double current_angular_velocity_z=0;
ros::Time last_imu_time;  // 上一次IMU消息时间
bool imu_first_msg = true;
bool check_L_0 =false;
bool check_L_1 =false;
bool local_corner_point = false;
//里程计数据
float odom_dist =0.0;
bool is_start =false;

double Time_local = 0.0;

//环岛状态机
int Round_step = 0;
float Round_step1_k=0.0;
//第一阶段角度
float angle_rad_step1=0.0;
float angle_deg_step1=0.0;
//
ros::Time Round_timer(0);

float Laser_linear_dis = 0.0;
bool Laser_dis_check = false;
int main(int argc , char ** argv)
{
    //初始化节点
    ros::init(argc,argv,"follow_end");
    //安装信号处理函数
    std::signal(SIGINT, signalHandler);
    ROS_INFO("Begin ImagePrespectiveInit ...");
    ImagePerspective_Init();
    ROS_INFO("End ImagePrespectiveInit ...");
    generateLookupTable(mapx,mapy);
    ROS_INFO("End generateLookTable ...");
    ROS_INFO("================================");


    //订阅节点信息
    ros::NodeHandle n;
    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    end_pub = nh.advertise<std_msgs::String>("follow_end", 10);
    ros::Subscriber lidar_sub = n.subscribe("/scan", 10, &_LaserCallback);
    ros::Subscriber cam_sub = n.subscribe("/usb_cam/image_raw", 1, &_CamCallback);
    ros::Subscriber sub = n.subscribe("/imu", 10, &_imuCallback);
    ros::Subscriber run_sub = n.subscribe("follow_begin", 10, &_beginCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, &_odomCallback);

    ros::Rate loop_rate(30);//睡眠频率
    Global_move_timer = ros::Time::now();
    //主循环
    while(ros::ok()&&!sig_INT.load())
    {
        ros::spinOnce();
        if(run_car)
        {
            follow_line();
        }

        //睡眠
        loop_rate.sleep();
    }
    
    cv::destroyAllWindows();
    ROS_INFO("All images processed !!!");
    return 0;
}