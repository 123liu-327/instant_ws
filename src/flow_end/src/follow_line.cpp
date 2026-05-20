#include "flow_end/follow.h"
#include <flow_end/Laser_linear.h>
#include <flow_end/Point_Process.h>
#include <flow_end/MatTransform.h>
#include <flow_end/corner_move.h>
#include <flow_end/PID.h>
#include <flow_end/process_image.h>

ros::Time Round_timer_dida(0);


void angle_Right(int id)
{
    int im1 = clip(id - (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
    int ip1 = clip(id + (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
    float conf = fabs(rpts1a[id]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
    ROS_INFO("Right conf : %f and Index : %d",conf,id);
}
void angle_Left(int id)
{
    int im1 = clip(id - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
    int ip1 = clip(id + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
    float conf = fabs(rpts0a[id]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;
    ROS_INFO("Left conf : %f and Index : %d",conf,id);
}




int follow_line()
{
    // static bool check_frame_buffer_linear = false;
    //     // 获取图像数据(需要加�????)
    ROS_INFO("cv::Mat local_frame;");
    cv::Mat local_frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        if (frame.empty())
            return -1;
        local_frame = frame.clone();
    }


    //环岛部分
    if(Round_step ==1)
    {
        //停车根据雷达信息调整位姿
        geometry_msgs::Twist Round_msg;
        double delta_angle = -angle_deg_step1;
        if(delta_angle>0)
        {
            delta_angle=0-delta_angle;
        }
        if(track_type == TRACK_RIGHT)
        {
            delta_angle = angle_deg_step1;
            if(delta_angle<0)
            {
                delta_angle=0-delta_angle;
            }
        }
        Round_msg.linear.x = 0;
        pre_yaw = current_yaw;
        double rotated_angle = 0.0;  // 记录已旋转角度
        Round_timer_dida = ros::Time::now();
        while(1)
        {
            pub.publish(Round_msg);
            ROS_INFO("Begin stepping into Round ...");
            ros::spinOnce();
            
            // // 计算当前已旋转的角度差（注意角度环绕）
            // double diff = current_yaw - pre_yaw;

            // // 将 diff 限制在 [-180, 180] 范围内
            // if (diff > 180)
            //     diff -= 360;
            // else if (diff < -180)
            //     diff += 360;

            rotated_angle = rotated_angle + (ros::Time::now() - Round_timer_dida).toSec() * curent_wz * 57.3;
            Round_timer_dida = ros::Time::now();

            ROS_WARN("Rotation complete! rotated_angle = %.2f°, target = %.2f°", rotated_angle, delta_angle);
            // 判断是否达到目标旋转角度，允许一定误差，比如2度以内
            if (std::abs(rotated_angle) >= std::abs(delta_angle) - 2.0)
            {
                ROS_WARN("Rotation complete! rotated_angle = %.2f°, target = %.2f°", rotated_angle, delta_angle);
                break;
            }

            // 计算剩余角度误差
            double angle_error = delta_angle - rotated_angle;

            // PID 控制计算角速度 (假设pid对象已定义且初始化)
            double angular_z = pid.compute(0, angle_error);

            // 填写控制命令，只旋转不前进
            Round_msg.angular.z = 1.2*angular_z;
            Round_msg.linear.x =angular_z*ROUND_R ;//0.2
            if(track_type == TRACK_RIGHT)
            {
                Round_msg.linear.x =-angular_z*ROUND_R ;//0.2
            }
            Round_msg.linear.y = -0.02;//-0.05
            if(track_type == TRACK_RIGHT)
            {
                Round_msg.linear.y=0.03;//0.20.0
            }
            // while(track_type == TRACK_RIGHT)
            // {
            //     ROS_INFO("Rotating... angle_error=%.2f°, angular_z=%.2f", angle_error, angular_z);
            // }

            ROS_INFO("Rotating... angle_error=%.2f°, angular_z=%.2f", angle_error, angular_z);
        }
  
            Round_msg.linear.x = 0.0;
            Round_msg.linear.y = 0.0;
            Round_msg.angular.z = 0.0;

            pub.publish(Round_msg);
            ROS_INFO("Adjust Round ending ...");
            if(Round_step== 1)
            {
                Round_step= 2;//第一状态达成
                Round_timer = ros::Time::now();
            } 
    }

    if(Round_step == 3)
    {
        //停车根据雷达信息调整位姿
        geometry_msgs::Twist Round_msg;
        Round_msg.linear.x = 0.0;
        Round_msg.linear.y = 0.0;
        Round_msg.angular.z = 0.0;
        pub.publish(Round_msg);  // 立即停下

        ros::Duration(0.5).sleep();//确保正确停车；
        //左移一下，防止压线
        ros::Time Round_local_timer = ros::Time::now();
        // while((ros::Time::now()-Round_timer).toSec()<=0.6)
        // {
        //     Round_msg.linear.x = 0.0;
        //     Round_msg.linear.y = 0;
        //     Round_msg.angular.z = 0.0;
        //     pub.publish(Round_msg); 
        // }
        // 设置旋转角度：左转为正方向
        double target_angle = -45.0;  // 左转 30°
        if(track_type == TRACK_RIGHT)
        {
            target_angle =48.0; 
        }
        double delta_angle = target_angle;

        // 记录初始朝向
        double yaw_start = current_yaw;
        double rotated_angle = 0.0;

        pre_yaw = yaw_start;

        Round_timer_dida = ros::Time::now();
        while (ros::ok())
        {
            pub.publish(Round_msg);
            ros::spinOnce();

            // // 计算旋转角度（考虑环绕）
            // double diff = current_yaw - pre_yaw;
            // if (diff > 180)
            //     diff -= 360;
            // else if (diff < -180)
            //     diff += 360;

            // rotated_angle = diff;
            rotated_angle = rotated_angle + (ros::Time::now() - Round_timer_dida).toSec() * curent_wz * 57.3;
            Round_timer_dida = ros::Time::now();


            // 判断是否达到目标角度
            if (std::abs(rotated_angle) >= std::abs(delta_angle) - 1.0)
            {
                ROS_INFO("Rotation complete: rotated = %.2f°, target = %.2f°", rotated_angle, delta_angle);
                break;
            }

            // 计算剩余角度误差
            double angle_error = delta_angle - rotated_angle;

            // 用 PID 控制角速度（你需要事先初始化 PID 对象）
            double angular_z = pid.compute(0.0, angle_error);

            // 限制最大角速度，防止过快（例如 ±0.5 rad/s）
            //angular_z = std::max(std::min(angular_z, 0.5), -0.3);
            //允许负值，不要只限制正方向（否则无法左转）
            angular_z = std::clamp(angular_z, -0.5, 0.5);

            // 发布指令
            Round_msg.linear.x = 0.01;
            if(track_type == TRACK_LEFT)
            {
                Round_msg.linear.y = 0.01;
            }
            else
            {
                Round_msg.linear.y = -0.01;
            }
            Round_msg.angular.z = angular_z;

            ROS_INFO("Rotating... rotated = %.2f°, error = %.2f°, angular_z = %.2f",
                    rotated_angle, angle_error, angular_z);

        }
        if(Round_step== 3)
        {
            Round_step= 4;//第二状态达成
            Round_timer = ros::Time::now();
        } 
    }

    if(Round_step == 5)
    {
        params.Kp = 1.2;
        geometry_msgs::Twist Round_msg;
        double delta_angle = -angle_deg_step1-160;
        if(track_type == TRACK_RIGHT)
        {
            delta_angle = angle_deg_step1+75;
        }
        Round_msg.linear.x = 0;
        pre_yaw = current_yaw;

        double rotated_angle = 0.0;  // 记录已旋转角度
        Round_timer_dida = ros::Time::now();
        while(1)
        {
            pub.publish(Round_msg);
            ROS_INFO("Begin stepping into Round ...");
               ros::spinOnce();

            // // 计算当前已旋转的角度差（注意角度环绕）
            // double diff = current_yaw - pre_yaw;

            // // 将 diff 限制在 [-180, 180] 范围内
            // if (diff > 180)
            //     diff -= 360;
            // else if (diff < -180)
            //     diff += 360;

            // rotated_angle = diff;

            rotated_angle = rotated_angle + (ros::Time::now() - Round_timer_dida).toSec() * curent_wz * 57.3;
            Round_timer_dida = ros::Time::now();


            // 判断是否达到目标旋转角度，允许一定误差，比如2度以内
            if (std::abs(rotated_angle) >= std::abs(delta_angle) - 2.0)
            {
                ROS_INFO("Rotation complete! rotated_angle = %.2f°, target = %.2f°", rotated_angle, delta_angle);
                break;
            }

            // 计算剩余角度误差
            double angle_error = delta_angle - rotated_angle;

            // PID 控制计算角速度 (假设pid对象已定义且初始化)
            double angular_z = pid.compute(0, angle_error);

            // 填写控制命令，只旋转不前进
            Round_msg.angular.z = angular_z;
            Round_msg.linear.x = 0.2;//0.2*0.
            Round_msg.linear.y = -0.05;

            if(track_type == TRACK_RIGHT)
            {
                Round_msg.linear.x = -angular_z*ROUND_R;//0.2angular_z*ROUND_R
                Round_msg.linear.y = 0.0;
                Round_msg.angular.z*=1.2;
            }

            ROS_INFO("Rotating... angle_error=%.2f°, angular_z=%.2f", angle_error, angular_z);
        }
  
            Round_msg.linear.x = 0.0;
            Round_msg.linear.y = 0.0;
            Round_msg.angular.z = 0.0;

            pub.publish(Round_msg);
            ROS_INFO("Adjust Round ending ...");
            if(Round_step== 5)
            {
                params.Kp = 0.05;
                Round_step= 6;//第三状态达成
                ros::Time local_corner = ros::Time::now();
                float timer_end =1.5;
                if(track_type==TRACK_LEFT)
                {
                    timer_end = 1.3;
                }
                while((ros::Time::now()-local_corner).toSec()<=timer_end)
                {
                      
                    Round_msg.linear.x = 0.0;
                    Round_msg.linear.y = 0.1;
                    Round_msg.angular.z = -0.5;
                    if(track_type==TRACK_LEFT)
                    {
                        Round_msg.angular.z = 0.5;
                        Round_msg.linear.y = -0.15;
                    }

                    pub.publish(Round_msg);
                }
                is_lidar_update = false;
                Round_timer = ros::Time::now();
            } 
    }

    if(is_lidar_update&&local_corner_point&&Round_step== 6&&(ros::Time::now()-Round_timer).toSec()>=4.0)//
    {
        ROS_INFO("Laser_Linear ...");
        Laser_linear();//直线
        return 0;
    }

    cv::Mat image_gray;
    cv::cvtColor(local_frame, image_gray, cv::COLOR_BGR2GRAY);
    convertMatTo2DArray(image_gray, PER_IMG);
    invertImage(PER_IMG);
    img_raw.data = PER_IMG[0];
    process_image();

    //检测角点
    Ypt1_found = false;
    Lpt1_found = false;
    Ypt0_found = false;
    Lpt0_found = false;
    is_straight0 = rpts0s_num > 1. / sample_dist;
    is_straight1 = rpts1s_num > 1. / sample_dist;
    for (int i = 0; i < rpts0s_num; i++)
    {
        if (rpts0an[i] == 0)
            continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts0s_num - 1);
        float conf = fabs(rpts0a[i]) - (fabs(rpts0a[im1]) + fabs(rpts0a[ip1])) / 2;
        // Y角点阈�?
        if (Ypt0_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / sample_dist)
        {
            Ypt0_rpts0s_id = i;
            Ypt0_found = true;
        }
        // L角点阈�?
        if (Lpt0_found == false && 70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 0.8 / sample_dist)
        {
            Lpt0_rpts0s_id = i;
            Lpt0_found = true;
        }
        // 长直道阈�????
        if (conf > 20.0 / 180. * PI && i < 1.0 / sample_dist)
            is_straight0 = false;
        if (Ypt0_found == true && Lpt0_found == true && is_straight0 == false)
            break;
    }
    for (int i = 0; i < rpts1s_num; i++)
    {
        if (rpts1an[i] == 0)
            continue;
        int im1 = clip(i - (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        int ip1 = clip(i + (int)round(angle_dist / sample_dist), 0, rpts1s_num - 1);
        float conf = fabs(rpts1a[i]) - (fabs(rpts1a[im1]) + fabs(rpts1a[ip1])) / 2;
        if (Ypt1_found == false && 30. / 180. * PI < conf && conf < 65. / 180. * PI && i < 0.8 / sample_dist)
        {
            Ypt1_rpts1s_id = i;
            Ypt1_found = true;
        }
        if (Lpt1_found == false && 70. / 180. * PI < conf && conf < 110. / 180. * PI && i < 0.8 / sample_dist)
        {
            Lpt1_rpts1s_id = i;
            Lpt1_found = true;
        }
        if (conf > 20.0 / 180. * PI && i < 1.0 / sample_dist)
            is_straight1 = false;
        if (Ypt1_found == true && Lpt1_found == true && is_straight1 == false)
            break;
    }
    
    //停车逻辑
    float corner_dot[2] = {0, 0};
    static int after_laser_cnt = 0;
    int max_laser_cnt = 5;
    //if (((Lpt1_found&&Lpt1_rpts1s_id>=5) || (Lpt0_found&&Lpt0_rpts0s_id>=5)) && check_after_laser && after_laser_cnt <= max_laser_cnt)
    if (((Lpt1_found) || (Lpt0_found)) && check_after_laser && after_laser_cnt <= max_laser_cnt)
    {
        after_laser_cnt = after_laser_cnt + 1;
        
    }else if (after_laser_cnt < 5&& check_after_laser){
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.angular.z = 0;
        pub.publish(msg);
        return 0;
    }
    //if ((after_laser_cnt > max_laser_cnt)||(Round_nosee_check&&after_laser_cnt >= 1))
    // if ((after_laser_cnt > max_laser_cnt)&&check_after_laser)
    //if ((after_laser_cnt > max_laser_cnt)||(Round_nosee_check&&after_laser_cnt >= 1))
    // if ((after_laser_cnt > max_laser_cnt)&&check_after_laser)
    if (check_after_laser&&(after_laser_cnt > max_laser_cnt))
    {
        bool is_stop_corner;
        is_stop_corner = false;
        if (Lpt1_found)
        {
            int im1 = clip(Lpt1_rpts1s_id - (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
            int ip1 = clip(Lpt1_rpts1s_id + (int)round(angle_dist / sample_dist), 0, rptsc1_num - 1);
            is_stop_corner = (rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1] > 20) && 
                             (rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0] < -20) &&
                             (rptsc1[Lpt1_rpts1s_id][1] > 480-40);         
            corner_move(rpts1s, corner_dot, Lpt1_rpts1s_id, -pixel_per_meter * (ROAD_WIDTH * 1.0) / 2);
            // ROS_WARN("is_stop_corner is %d, Lpt1_found is %d", is_stop_corner, Lpt1_found);
            // ROS_WARN("rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1] %f", rptsc1[im1][1] - rptsc1[Lpt1_rpts1s_id][1]);
            // ROS_WARN("rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0] %f", rptsc1[ip1][0] - rptsc1[Lpt1_rpts1s_id][0]);
            // ROS_WARN("rptsc1[Lpt1_rpts1s_id][1] %f", rptsc1[Lpt1_rpts1s_id][1]);
            // ROS_WARN("rptsc1[Lpt1_rpts1s_id][1] %f", rptsc1[Lpt1_rpts1s_id][1]);
        }
        else if (Lpt0_found)
        {
            int im0 = clip(Lpt0_rpts0s_id - (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
            int ip0 = clip(Lpt0_rpts0s_id + (int)round(angle_dist / sample_dist), 0, rptsc0_num - 1);
            is_stop_corner = (rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1] > 20) && 
                             (rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0] > 20) &&
                             (rptsc0[Lpt0_rpts0s_id][1] > 480-40);     
            corner_move(rpts0s, corner_dot, Lpt0_rpts0s_id, pixel_per_meter * (ROAD_WIDTH * 1.0) / 2);
            // ROS_WARN("is_stop_corner is %d, Lpt0_found is %d", is_stop_corner, Lpt0_found);
            // ROS_WARN("rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1] %f", rptsc0[im0][1] - rptsc0[Lpt0_rpts0s_id][1]);
            // ROS_WARN("rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0] %f", rptsc0[ip0][0] - rptsc0[Lpt0_rpts0s_id][0]);
            // ROS_WARN("rptsc0[Lpt0_rpts0s_id][1] %f", rptsc0[Lpt0_rpts0s_id][1]);
            // ROS_WARN("rptsc0[Lpt0_rpts0s_id][1] %f", rptsc0[Lpt0_rpts0s_id][1]);
        }
        if ((Lpt1_found || Lpt0_found)&&is_stop_corner){
            float cx = 320, cy = 490;
            float delta_cx = corner_dot[0] - cx;
            float delta_cy = corner_dot[1] - cy;
            float target_dis = -delta_cy / pixel_per_meter;
            float target_dis_x = -delta_cx / pixel_per_meter;
            ros::Time start_time = ros::Time::now();
            while (1)
            {
                ros::spinOnce();
                double delta_angle = (current_yaw - pre_yaw);
                if (delta_angle > 360)
                {
                    delta_angle = delta_angle - 720;
                }
                else if (delta_angle < -360)
                {
                    delta_angle = delta_angle + 720;
                }
                //ROS_INFO("hhhhh: delta_angle=%.3f, pre_angle_deg=%.3f", delta_angle, pre_angle_deg);
                delta_angle = delta_angle + pre_angle_deg;
                double output = pid.compute(0, delta_angle);
                geometry_msgs::Twist local_msg;
                local_msg.linear.x = 0.15;
                if (std::abs(target_dis_x)<0.08){
                    local_msg.linear.y = 0.0;
                }else{
                    if (target_dis_x > 0){
                        local_msg.linear.y = 0.1;
                    }else{
                        local_msg.linear.y = -0.1;
                    }
                }
                local_msg.angular.z = 0.0;//output
                //ROS_INFO("hhhhh: output=%.3f", output);
                float time = (ros::Time::now() - start_time).toSec();
                start_time = ros::Time::now();
                float dis = -0.215;
                target_dis_x = target_dis_x - local_msg.linear.y * time;
                if (target_dis < dis)
                {
                    // ROS_INFO("Stop car\n");

                    // local_msg.linear.x = 0.0;
                    // local_msg.linear.z = 0.0;
                    // std_msgs::String msg_end;
                    // msg_end.data = "STOP";    // 设置消息内容
                    // end_pub.publish(msg_end); // 发布消息
                    // kill(getpid(),SIGINT);
                    ROS_INFO("Stop car\n");

                    local_msg.linear.x = 0.0;
                    local_msg.angular.z = 0.0;
                    for (int i = 0; i < 10; i++) {
                        pub.publish(local_msg);
                        ros::Duration(0.1).sleep();
                    }
                    std_msgs::String msg_end;
                    msg_end.data = "STOP";
                    for (int i = 0; i < 10; i++) {
                        end_pub.publish(msg_end);
                        ros::Duration(0.1).sleep();
                    }
                    ros::shutdown();  // 优雅退出
                    
                }

                target_dis = target_dis - local_msg.linear.x * time;
                ROS_INFO("target_dis: %f", target_dis);
                pub.publish(local_msg);
                
            }
        }

    }
    angle_Left(Lpt0_rpts0s_id );
    angle_Right(Lpt1_rpts1s_id);
    //ROS_INFO("after_laser_cnt: %d",after_laser_cnt);
    ROS_INFO("Ypt0_found: %d, Lpt0_found: %d, is_straight0: %d, ", Ypt0_found, Lpt0_found, is_straight0);
    ROS_INFO("Ypt1_found: %d, Lpt1_found: %d, is_straight1: %d", Ypt1_found, Lpt1_found, is_straight1);
    ROS_INFO("hhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhhh\n");
    float cx = 320, cy = 490;
    float delta_cx = corner_dot[0] - cx;
    float delta_cy = corner_dot[1] - cy;
    float error = 0, v = 0;
    if(Lpt0_found)
    {
        check_L_0=true;
    }
    if(Lpt1_found)
    {
        check_L_1=true;
    }


    // 巡线形式
    // track_type = TRACK_LEFT;
    rpts_num = 0;
    static float middle_rpts[POINTS_MAX_LEN][2];
    int middle_rpts_num = 0;
    if (track_type == TRACK_LEFT)
    {
        if (rptsc0e_num != 0)
        {
            rpts = rptsc0e;
            rpts_num = rptsc0e_num;
        }
        else
        {
            rpts = rptsc1e;
            rpts_num = rptsc1e_num;
        }
    }
    else if (track_type == TRACK_RIGHT)
    {
        if (rptsc1e_num != 0)
        {
            rpts = rptsc1e;
            rpts_num = rptsc1e_num;
        }
        else
        {
            rpts = rptsc0e;
            rpts_num = rptsc0e_num;
        }
    }
    else
    {
        if (rptsc0e_num != 0 && rptsc1e_num != 0)
        {
            middle_rpts_num = std::min(rptsc0e_num, rptsc1e_num);
            for (int i = 0; i < middle_rpts_num; i++)
            {
                middle_rpts[i][0] = (rptsc0e[i][0] + rptsc1e[i][0]) * 0.5f;
                middle_rpts[i][1] = (rptsc0e[i][1] + rptsc1e[i][1]) * 0.5f;
            }
            rpts = middle_rpts;
            rpts_num = middle_rpts_num;
        }
        else if (rptsc1e_num != 0)
        {
            rpts = rptsc1e;
            rpts_num = rptsc1e_num;
        }
        else
        {
            rpts = rptsc0e;
            rpts_num = rptsc0e_num;
        }
    }
    float v_y = 0.0;
    if (rpts_num == 0)
    {
        if (check_after_laser)
        {
            error = 0.0;
            v = 0.2;

            zeroCount = zeroCount + 1; 
            if (zeroCount >= 2)
            {
                zero_flag = true;
            }
            if (zero_flag)
            {
                error = 0.0;
                v = 0.0;
            }
        }
        else
        {
            if (track_type == TRACK_LEFT)
            {
                if(Round_step== 6)
                {
                    error = 0.3;
                    v_y=0.07;
                }
            }

            else
            {
                
                if(Round_step== 6)
                {
                    error = -0.3;
                    v_y=-0.07;
                }
            }
                
            v = 0.2;
        

        }
    }
    else
    {
        if (check_after_laser)
        {
            zeroCount = 0; // ֻҪ��һ֡��Ϊ0�������ü�����
            int aim_idx = clip(round(0.1 / sample_dist), 0, rpts_num - 1);
            float dx = rpts[aim_idx][0] - cx;
            float dy = cy - rpts[aim_idx][1] + 0.2 * pixel_per_meter;

            float dn = sqrt(dx * dx + dy * dy);
            error = -atan2f(dx, dy) * 1.0;
            v = 0.2 - std::abs(error) * 0.2;
        }
        else
        {
            int aim_idx = clip(round(0.1 / sample_dist), 0, rpts_num - 1);
            float dx = rpts[aim_idx][0] - cx;
            float dy = cy - rpts[aim_idx][1] + 0.2 * pixel_per_meter;

            float dn = sqrt(dx * dx + dy * dy);
            error = -atan2f(dx, dy) * 1.0;
            v = 0.3 - std::abs(error) * 0.3;
            ROS_INFO("xxxxxx: error=%.3f, v=%.3f", error, v);
        }
    }


    if(check_L_0&&check_L_1&&!local_corner_point&& rpts_num == 0)//
    {
       local_corner_point = true;
       ros::Time local_corner_timer = ros::Time::now();
       while((ros::Time::now()-local_corner_timer).toSec()<=Time_local)
       {
            geometry_msgs::Twist corner_msg;
            corner_msg.linear.x = v;
            corner_msg.angular.z = error;
            pub.publish(corner_msg);
       }
       return 0;
    }           


    // std::cout  << "error:" << error << std::endl;
    ROS_INFO("error : %f\n", error);
    geometry_msgs::Twist msg;
    msg.linear.x = v;
    if (check_after_laser && std::abs(error)>0.1){
        error = 0;
    }
    if(check_after_laser)
    {
        v_y=0.0;
    }
    msg.angular.z = -error;
    msg.linear.y = v_y;
    pub.publish(msg);

    invertImage(PER_IMG);
    for (int i = 0; i < RESULT_ROW; i++)
    {
        for (int j = 0; j < RESULT_COL; j++)
        {
            img_line_data[i][j] = ImageUsed[i][j];
        }
    }
    for (int i = 0; i < rptsc0e_num; i++)
    {
        AT_IMAGE(&img_line, clip(rptsc0e[i][0], 0, img_line.width - 1),
                 clip(rptsc0e[i][1], 0, img_line.height - 1)) = 0;
    }
    for (int i = 0; i < rptsc1e_num; i++)
    {
        AT_IMAGE(&img_line, clip(rptsc1e[i][0], 0, img_line.width - 1),
                 clip(rptsc1e[i][1], 0, img_line.height - 1)) = 80;
    }
    for (int i = 0; i < rpts0s_num; i++)
    {
        AT_IMAGE(&img_line, clip(rpts0s[i][0], 0, img_line.width - 1),
                 clip(rpts0s[i][1], 0, img_line.height - 1)) = 0;
    }
    for (int i = 0; i < rpts1s_num; i++)
    {
        AT_IMAGE(&img_line, clip(rpts1s[i][0], 0, img_line.width - 1),
                 clip(rpts1s[i][1], 0, img_line.height - 1)) = 80;
    }
    cv::Mat converted_image = convert2DArrayToMat(img_line_data);

    cv::imshow("Image", converted_image);

    if (cv::waitKey(1) == 'q')
    {
        return -1;
    }

    return 0;
}
