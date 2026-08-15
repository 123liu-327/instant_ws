#include<flow_end/Callback.h>
#include <flow_end/follow_line.h>
#include <flow_end/Laser_linear.h>
#include <flow_end/Point_Process.h>
#include <flow_end/MatTransform.h>
#include <flow_end/corner_move.h>
#include <flow_end/PID.h>
#include <flow_end/follow.h>

void Laser_Linear_callback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    static float Angle_begin = -85.0;
    static float Angle_end = -95.0; 
    const float ANGLE_START = Angle_begin * M_PI / 180.0;  
    const float ANGLE_END = Angle_end  * M_PI / 180.0;                   

    const int FILTER_WINDOW = 5;     //滤波窗口大小必须是奇数                
    //存储数据
    std::vector<float>select_ranges;

    const float right_angle_start =ANGLE_START ; 
    const float right_angle_end   = ANGLE_END  ; 

    int start_idx = std::round((right_angle_start - scan->angle_min) / scan->angle_increment);
    int end_idx   = std::round((right_angle_end   - scan->angle_min) / scan->angle_increment);

    // 安全边界限制（避免越界）
    start_idx = std::max(0, start_idx);
    end_idx = std::min((int)scan->ranges.size() - 1, end_idx);

    // 打印角度范围、索引范围、总点数
    ROS_INFO("雷达角度范围: [%.2f°, %.2f°], 对应索引范围: [%d, %d], 总点数: %lu",
         right_angle_start * 180.0 / M_PI,
         right_angle_end   * 180.0 / M_PI,
         start_idx, end_idx,
         scan->ranges.size());

        // 提取右侧区域的有效数据
    for (int i = start_idx; i >= end_idx; --i)//如果索引是反向索引，采用第一种方式进行存储
    {
        float range = scan->ranges[i];
        if (!std::isinf(range) && !std::isnan(range))
        {
            select_ranges.push_back(range);
        }
    }

    if (select_ranges.empty())//如果索引是正向索引，采用如下循环进行储存
    {
        ROS_WARN("Empty right laser scan ...");
        for (int i = start_idx; i <= end_idx; i++)
        {
            float range = scan->ranges[i];
            if (!std::isinf(range) && !std::isnan(range))
            {
                select_ranges.push_back(range);
            }
        }
    }


        // 滑动均值滤波
    std::vector<float> filtered_ranges;
    int N = select_ranges.size();
    int half_win = FILTER_WINDOW / 2;

    for (int i = 0; i < N; ++i)
    {
        int start = std::max(0, i - half_win);
        int end = std::min(N - 1, i + half_win);
        float sum = 0.0;
        int count = 0;

        for (int j = start; j <= end; ++j)
        {
            sum += select_ranges[j];
            count++;
        }
        filtered_ranges.push_back(sum / count);
            // === 计算滤波后的整体均值 ===
        if (!filtered_ranges.empty())
        {
            float total = 0.0;
            for (float val : filtered_ranges)
                total += val;

            float avg = total / filtered_ranges.size();
            Laser_linear_dis = avg;

            ROS_INFO("滤波后距离平均值: %.2f m", avg);
        }
        else
        {
            ROS_WARN("滤波后没有有效数据.");
        }
    }
}



//基于雷达判断第一阶段是否进入环岛

void _LaserRound(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    static float Angle_begin = 0.0;
    static float Angle_end = 0.0;
    float DIST_THRESHOLD_MIN = 1.4;   //距离阈值  
    float DIST_THRESHOLD_MAX =1.6 ; 
    ROS_INFO("Round_step:[%d]",Round_step);
    if(track_type == TRACK_LEFT)
    {
        if(Round_step == 0)//||Round_step ==6
        {
            Angle_begin=-60.0;
            Angle_end = -105.0;
            DIST_THRESHOLD_MIN =0.95;   //距离阈值  
            DIST_THRESHOLD_MAX =1.05; 
        }
        if(Round_step==2)
        {
            Angle_begin=15.0;
            Angle_end = 45.0;
            DIST_THRESHOLD_MIN = 1.1;   //距离阈值  
            DIST_THRESHOLD_MAX =1.2 ; 
        }
        if(Round_step==4)
        {
            Angle_begin=50.0;
            Angle_end =65.0;
            DIST_THRESHOLD_MIN =1.6;   //距离阈值  
            DIST_THRESHOLD_MAX =1.7 ; 
        }
    }
    if(track_type == TRACK_RIGHT)
    {
        if(Round_step == 0&&(ros::Time::now()-Global_move_timer).toSec()>=2.0)//||Round_step ==6
        {
            Angle_begin=-30.0;
            Angle_end = -50.0;
            DIST_THRESHOLD_MIN = 1.6;   //距离阈值  
            DIST_THRESHOLD_MAX =1.65 ; 
        }
        if(Round_step==2)
        {
            Angle_begin=-10.0;
            Angle_end =-50.0;
            DIST_THRESHOLD_MIN =1.05;   //距离阈值  
            DIST_THRESHOLD_MAX =1.1; 
        }
        if(Round_step==4)
        {
            Angle_begin=-140;
            Angle_end = -179;
            DIST_THRESHOLD_MIN = 1.3;   //距离阈值  
            DIST_THRESHOLD_MAX =1.4 ; 
        }
    }


    const float ANGLE_START = Angle_begin * M_PI / 180.0;  
    const float ANGLE_END = Angle_end  * M_PI / 180.0;                   

    const int FILTER_WINDOW = 5;     //滤波窗口大小必须是奇数                
    //存储数据
    std::vector<float>select_ranges;

    const float right_angle_start =ANGLE_START ; 
    const float right_angle_end   = ANGLE_END  ; 

    int start_idx = std::round((right_angle_start - scan->angle_min) / scan->angle_increment);
    int end_idx   = std::round((right_angle_end   - scan->angle_min) / scan->angle_increment);

    // 安全边界限制（避免越界）
    start_idx = std::max(0, start_idx);
    end_idx = std::min((int)scan->ranges.size() - 1, end_idx);

    // 打印角度范围、索引范围、总点数
    // ROS_INFO("雷达角度范围: [%.2f°, %.2f°], 对应索引范围: [%d, %d], 总点数: %lu",
    //      right_angle_start * 180.0 / M_PI,
    //      right_angle_end   * 180.0 / M_PI,
    //      start_idx, end_idx,
    //      scan->ranges.size());

        // 提取右侧区域的有效数据
    for (int i = start_idx; i >= end_idx; --i)//如果索引是反向索引，采用第一种方式进行存储
    {
        float range = scan->ranges[i];
        if (!std::isinf(range) && !std::isnan(range))
        {
            select_ranges.push_back(range);
        }
    }

    if (select_ranges.empty())//如果索引是正向索引，采用如下循环进行储存
    {
        //ROS_WARN("Empty right laser scan ...");
        for (int i = start_idx; i <= end_idx; i++)
        {
            float range = scan->ranges[i];
            if (!std::isinf(range) && !std::isnan(range))
            {
                select_ranges.push_back(range);
            }
        }
    }


        // 滑动均值滤波
    std::vector<float> filtered_ranges;
    int N = select_ranges.size();
    int half_win = FILTER_WINDOW / 2;

    for (int i = 0; i < N; ++i)
    {
        int start = std::max(0, i - half_win);
        int end = std::min(N - 1, i + half_win);
        float sum = 0.0;
        int count = 0;

        for (int j = start; j <= end; ++j)
        {
            sum += select_ranges[j];
            count++;
        }

        float avg = sum / count;
        filtered_ranges.push_back(avg);
    }
    float dis_min=0.0;
    // 判断是否存在障碍物
    bool obstacle_found = false;
    if(!filtered_ranges.empty())
    {
         dis_min=filtered_ranges[0];
    }

    for (int i = 0; i < filtered_ranges.size(); ++i)
    {
        if(dis_min>filtered_ranges[i])
        {
            dis_min=filtered_ranges[i];
        }
    }
        if (dis_min >=  DIST_THRESHOLD_MIN&&dis_min<=DIST_THRESHOLD_MAX)
        {
            obstacle_found = true;
        }
    if(obstacle_found&&Round_step == 0)
    {
        Round_step = 1;//进入第一个状态，停车调整位姿
    }

    if(obstacle_found&&Round_step == 2&&(ros::Time::now()-Round_timer).toSec()>=2.5)
    {
        Round_step = 3;
    }

    if(obstacle_found&&Round_step == 4&&(ros::Time::now()-Round_timer).toSec()>=1.7)
    {
        Round_step = 5;
    }
    

    //将坐标转换成笛卡尔坐标，并使用最小二乘法计算斜率
     // 计算极坐标转换为直角坐标，拟合斜率
    std::vector<float> x_points;
    std::vector<float> y_points;

    for (int i = 0; i < filtered_ranges.size(); ++i)
    {
        int idx = start_idx + i;
        float angle = scan->angle_min + idx * scan->angle_increment;
        float r = filtered_ranges[i];

        // 极坐标转直角坐标
        float x = r * cos(angle);
        float y = r * sin(angle);

        x_points.push_back(x);
        y_points.push_back(y);
    }

    // 线性拟合计算斜率（最小二乘法）
    float sum_x = 0.0, sum_y = 0.0, sum_xx = 0.0, sum_xy = 0.0;
    int n = x_points.size();

    for (int i = 0; i < n; ++i)
    {
        float x = x_points[i];
        float y = y_points[i];
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }
    if (n >= 2)
    {
        float denominator = (n * sum_xx - sum_x * sum_x);
        if (fabs(denominator) > 1e-6)
        {
            Round_step1_k= (n * sum_xy - sum_x * sum_y) / denominator;
            // 将斜率转为角度
            angle_rad_step1 = std::atan(Round_step1_k);                   // 单位：弧度
            angle_deg_step1 = angle_rad_step1 * 180.0 / M_PI;       // 单位：角度
            //ROS_INFO("Estimated slope (Round_step1_k): %.4f",Round_step1_k);
            //ROS_INFO("Estimated angle (deg): %.2f", angle_deg_step1);
        }

    }

}




//雷达回调函数实现
void _LaserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    //环岛第一部分检查
    if(run_car)
    {
        _LaserRound(scan);
        Laser_Linear_callback(scan);
    }

    // 参数校验
    if (scan->ranges.empty())
    {
        ROS_WARN("Empty laser scan received!");
        return;
    }

    // 计算前方±15°区域
    const float front_angle_rad = 15.0 * M_PI / 180.0;
    int center_idx = round((-scan->angle_min - (5.0 * M_PI / 180.0)) / scan->angle_increment);
    int range = round(front_angle_rad / scan->angle_increment);

    //滤波出有效数据
    std::vector<Laser> valid_distances;
    for (int i = center_idx - range; i <= center_idx + range; ++i)
    {
        if (i >= 0 && i < scan->ranges.size())
        {
            float dist = scan->ranges[i];
            if (!std::isnan(dist) &&
                dist >= scan->range_min &&
                dist <= scan->range_max)
            {
                Laser laser_data;
                laser_data.data = dist;
                laser_data.index = i;
                valid_distances.push_back(laser_data);
            }
        }
    }

    // 安全判断
    if (!valid_distances.empty())
    {
        float min_dist = valid_distances[0].data;
        float min_dist_check = valid_distances[0].data;
        int min_dist_index = valid_distances[0].index;
        float max_dist = valid_distances[valid_distances.size() - 1].data;
        int max_dist_index = valid_distances[valid_distances.size() - 1].index;

        for (int i = 0; i < valid_distances.size(); i++)
        {
            if (valid_distances[i].data < min_dist_check)
            {
                min_dist_check = valid_distances[i].data;
                Dist_1 =min_dist_check;
            }
        }

        //计算出对应角度
        float min_angle = scan->angle_min + min_dist_index * scan->angle_increment - M_PI / 2;
        float max_angle = scan->angle_min + max_dist_index * scan->angle_increment - M_PI / 2;

        //由极坐标系转换为平面坐标系
        float min_x = min_dist * cos(min_angle);
        float min_y = min_dist * sin(min_angle);

        float max_x = max_dist * cos(max_angle);
        float max_y = max_dist * sin(max_angle);

        // 计算斜率
        slope = 0.0;
        if (fabs(max_x - min_x) > 0.001)
        {
            slope = (max_y - min_y) / (max_x - min_x);
        }
        else
        {
            slope = (max_y > min_y) ? INFINITY : -INFINITY;
        }

        //将斜率转换成对应角度
        angle_deg = atan2(max_y - min_y, max_x - min_x) * 180.0 / M_PI;
        //做偏置处理
        angle_deg = angle_deg - ANGLE_BIAS;
        current_yaw_lidar = angle_deg;
        ROS_INFO("Min(%.2f, %.2f), Max(%.2f, %.2f), k: %.2f, angle: %.2f°",
                 min_x, min_y, max_x, max_y, slope, angle_deg);
        //判断是否触发避障条件
        if (run_car == true && Dist_1 < LASE_MIN)
        {
            is_lidar_update = true;
        }
        ROS_INFO("Laser is fine!");
    }
}

//摄像头回调函数
void _CamCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        //将ROS图像信息转换为OPENCV形式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        //互斥进入临界区，防止竞态条件
        {
            std::lock_guard<std::mutex> lock(frame_mutex); // 使用lock_guard替代unique_lock
            frame = cv_ptr->image.clone();                 // 显式深拷�????
        } //c++自动解锁
        ROS_INFO("cv_bridge is fine !\n");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

//IMU回调函数
void _imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // 提取四元数
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

    // 将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 转换为角度
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;
    current_yaw = yaw;
    curent_wz = msg->angular_velocity.z;
    //角速度和线速度获取
    current_angular_velocity_z = msg->angular_velocity.z;

    // 时间差计算
    ros::Time now = ros::Time::now();
    if (imu_first_msg) {
        last_imu_time = now;
        imu_first_msg = false;
        return;
    }
    double dt = (now - last_imu_time).toSec();
    last_imu_time = now;

    // 线加速度（单位：m/s^2）
    double ax = msg->linear_acceleration.x;

    // // 可选：滤波小加速度（如小于0.05认为是0，去除噪声）
    // if (std::abs(ax) < 0.05) {
    //     ax = 0;
    // }

    // 积分计算线速度：v = v0 + a * dt
    current_linear_velocity_x += ax * dt;

    // 可选：防止速度长期累加漂移，加入阻尼项
    current_linear_velocity_x *= 0.99;//0.99

    // 打印结果
    ROS_INFO("Yaw: %.2f, Pitch: %.2f, Roll: %.2f | ω_z: %.2f | v_x: %.2f", 
             yaw, pitch, roll, current_angular_velocity_z, current_linear_velocity_x);

    // 打印结果
    ROS_INFO("current_yaw: %.2f, Pitch: %.2f, Yaw: %.2f", current_yaw, pitch, yaw);

}
//讯息起始信息回调
void _beginCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == "Left")
    {
        track_type = TRACK_LEFT;
        run_car = true;
        Dis_Bias_Left = 6.0;
        Dis_Bias_Right = 0.0;
        Time_local = 0.5;
    }
    else if (msg->data == "Right")
    {
        track_type = TRACK_RIGHT;
        run_car = true;
        Dis_Bias_Left = 0.0;
        Dis_Bias_Right = -15.0;
        Time_local = 0.5;
    }
    else if (msg->data == "Middle" || msg->data == "middle" ||
             msg->data == "Center" || msg->data == "center")
    {
        // 居中巡线不额外偏左/偏右，左右线生成的目标线都按 ROAD_WIDTH / 2 处理。
        // track_type 仍设为 TRACK_RIGHT，仅作为后续选择 rpts 时的默认侧。
        
        track_type = TRACK_MIDDLE;
        run_car = true;
        Dis_Bias_Left = 0.0;
        Dis_Bias_Right = 0.0;
        Time_local = 0.5;
    }
    else
    {
        ROS_WARN("Unknown command: %s", msg->data.c_str());
    }
    ROS_INFO("begin is fine !\n");
}

void _odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    float x0 = 0.0f;
    float y0 = 0.0f;
    float x_now = msg->pose.pose.position.x;
    float y_now = msg->pose.pose.position.y;

    if (!is_start)
    {
        x0 = x_now;
        y0 = y_now;
        is_start = true;
        return;
    }

    float dx = x_now - x0;
    float dy = y_now - y0;
    odom_dist = sqrt(dx * dx + dy * dy);  // 实际走的距离
}
