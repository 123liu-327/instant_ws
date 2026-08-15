#include <flow_end/Laser_linear.h>
#include <flow_end/PID.h>

void Laser_linear()
{
    geometry_msgs::Twist vel_cmd;
    if (!is_lidar_update)
    {
        return;
    }
    if (is_lidar_update)
    {
        
        float Time_laser_left = 1.6;
        float Time_laser_right = 1.59;
        if (track_type == TRACK_RIGHT)
        {
            Time_laser_left = 1.6;
            Time_laser_right = 1.6;
        }
        while (1)
        {
            ROS_INFO("Laser_linear while 111");

            if (check == 0 && Dist_1 < LASE_MIN)
            {
                vel_cmd.linear.x = 0.0;
                vel_cmd.linear.y = 0.3;  
                vel_cmd.angular.z = 0.0; 
                pre_yaw = current_yaw;
                pre_angle_deg = angle_deg;
                while (1)
                {
                    ros::spinOnce();
                    double delta_angle = -angle_deg;
                    if (delta_angle > 360)
                    {
                        delta_angle = delta_angle - 720;
                    }
                    else if (delta_angle < -360)
                    {
                        delta_angle = delta_angle + 720;
                    }
                    ROS_INFO("hhhhh: delta_angle=%.3f, pre_angle_deg=%.3f, current_yaw=%.3f, pre_yaw=%.3f", delta_angle, pre_angle_deg, current_yaw, pre_yaw);
                    double output = pid.compute(0, delta_angle);
                    geometry_msgs::Twist local_msg;
                    local_msg.linear.x = 0;
                    local_msg.linear.y = 0;
                    local_msg.angular.z = output;
                    // local_msg.angular.z = 0;
                    pub.publish(local_msg);
                    // ROS_INFO("hhhhh: output=%.3f", output);
                    if (std::abs(delta_angle) < 2)
                    {
                        check_imu = true;
                        pre_yaw = current_yaw;
                        break;
                    }
                }
                move_start_time = ros::Time::now();
                check = 1;
                ROS_INFO("Going left");
            }
            else if (check == 1 && (ros::Time::now() - move_start_time).toSec() > Time_laser_left)
            {
                // 左移 1 秒后，恢复直�????
                vel_cmd.linear.x = 0.3;
                vel_cmd.linear.y = 0.0;
                vel_cmd.angular.z = 0.0;
                move_start_time = ros::Time::now();
                check = 2;
                ROS_INFO("Going straight after left");
            }
            else if (check == 2 && (ros::Time::now() - move_start_time).toSec() > 2.5)
            {
                // 直�?? 1.5 秒后，开始右�????
                vel_cmd.linear.x = 0.0;
                vel_cmd.linear.y = -0.3; // �????向右移（如果底盘�????持）
                // vel_cmd.angular.z = -0.3; // 如果不支持横向移�????，改用转�????
                move_start_time = ros::Time::now();
                check = 3;
                ROS_INFO("Going right");
            }
            else if (check == 3 && (ros::Time::now() - move_start_time).toSec() > Time_laser_right)
            {
                is_lidar_update = false;
                // after_bizhang_x = -20;//避障之后改变检索图像的范围
                // after_bizhang_y = 40;
                check_after_laser = true;
                break;
            }
            pub.publish(vel_cmd);

        }
    }
    // while(!frame_buffer.empty())
    // {
    //     frame_buffer.pop_front();
    // }
    move_start_time = ros::Time::now();
    if(track_type==TRACK_LEFT)
    {
        Dis_Bias_Left = 20;
        Dis_Bias_Right = 0.0;
    }
    else
    {
        Dis_Bias_Left = 0.0;
        Dis_Bias_Right = 0.0;
    }
    // while((ros::Time::now() - move_start_time).toSec()<0.8)
    // {
    //     vel_cmd.linear.x = 0.3;
    //     vel_cmd.linear.y = 0.0;
    //     vel_cmd.angular.z = 0.0;
    //     pub.publish(vel_cmd);
    // }
    return;
}


// void Laser_linear()
// {
//     geometry_msgs::Twist vel_cmd;
//     if (!(is_lidar_update))
//     {
//         return;
//     }
//     if (is_lidar_update)
//     {
        
//         while (1)
//         {
//             ros::spinOnce();
//             ROS_INFO("Laser_linear while 111");
            
//             if (check == 0 )
//             {
//                 vel_cmd.linear.x = 0.0;
//                 vel_cmd.linear.y = 0.0;  
//                 vel_cmd.angular.z = 0.0; 
//                 pub.publish(vel_cmd);
//                 pre_yaw = current_yaw;
//                 pre_angle_deg = angle_deg;
//                 while (1)
//                 {
//                     ros::spinOnce();
//                     double delta_angle = -angle_deg;
//                     if (delta_angle > 360)
//                     {
//                         delta_angle = delta_angle - 720;
//                     }
//                     else if (delta_angle < -360)
//                     {
//                         delta_angle = delta_angle + 720;
//                     }
//                     ROS_INFO("hhhhh: delta_angle=%.3f, pre_angle_deg=%.3f, current_yaw=%.3f, pre_yaw=%.3f", delta_angle, pre_angle_deg, current_yaw, pre_yaw);
//                     double output = pid.compute(0, delta_angle);
//                     geometry_msgs::Twist local_msg;
//                     local_msg.linear.x = 0;
//                     local_msg.linear.y = 0;
//                     local_msg.angular.z = output;
//                     // local_msg.angular.z = 0;
//                     pub.publish(local_msg);
//                     // ROS_INFO("hhhhh: output=%.3f", output);
//                     if (std::abs(delta_angle) < 2)
//                     {
//                         check_imu = true;
//                         pre_yaw = current_yaw;
//                         break;
//                     }
//                 }
//                 ros::spinOnce();
//                 params.Kp = 0.5;
//                 double target_distance =  Laser_linear_dis+0.5;
//                 while(1)
//                 {
//                     // === PID 控制逻辑 ===
//                     ros::spinOnce();
//                     double error = target_distance - Laser_linear_dis; // 差值，期望 - 实际
//                     double output = pid.compute(0, error); // 设定值是 0，error 表示我们期望误差为 0
//                         // 限制最小速度阈值（反抖动），避免输出太小
//                     if (std::abs(output) < 0.05)
//                         output = (output > 0) ? 0.05 : -0.05;

//                     // 限幅最大速度
//                     output = std::clamp(output, -0.4, 0.4); // 可适当放大速度限制
//                     geometry_msgs::Twist cmd;
//                     output*=5;
//                     vel_cmd.linear.x = 0.0;
//                     vel_cmd.linear.y = -output;  
//                     vel_cmd.angular.z = 0.0;        // 保持不转弯

//                     pub.publish(vel_cmd);

//                     ROS_INFO("当前距离: %.3f, 目标: %.2f, 误差: %.3f, PID输出: %.3f", Laser_linear_dis, target_distance, error, output);

//                     // 若距离误差很小则判断完成
//                     if (std::abs(error) < 0.02)
//                     {
//                         ROS_INFO("距离控制已完成.");
//                         break;
//                     }
                    
//                 }
//                 //move_start_time = ros::Time::now();
//                 check = 1;
//                 ROS_INFO("Going left");
//             }
//             else if (check == 1 )
//             {   move_start_time= ros::Time::now();
//                 while((ros::Time::now()-move_start_time).toSec()<=2.3)
//                 {
//                     vel_cmd.linear.x = 0.3;
//                     vel_cmd.linear.y = 0.0;
//                     vel_cmd.angular.z = 0.0;
                    
//                     pub.publish(vel_cmd);
//                 }
//                 // 左移 1 秒后，恢复直�????

//                 check = 2;
//                 ROS_INFO("Going straight after left");
//             }
//             else if (check == 2)
//             {
//                 ros::spinOnce();
//                 double target_distance =  Laser_linear_dis-0.5;
//                 while(1)
//                 {
//                     ros::Time Time_laser = ros::Time::now();
//                     // === PID 控制逻辑 ===
//                     ros::spinOnce();
//                     double error = target_distance - Laser_linear_dis; // 差值，期望 - 实际
//                     double output = pid.compute(0, error); // 设定值是 0，error 表示我们期望误差为 0
//                     if (std::abs(output) < 0.05)
//                         output = (output > 0) ? 0.05 : -0.05;

//                     // 限幅最大速度
//                     output = std::clamp(output, -0.4, 0.4); // 可适当放大速度限制
//                     geometry_msgs::Twist cmd;
//                     output*=5;
//                     vel_cmd.linear.x = 0.0;
//                     vel_cmd.linear.y = -output;  
//                     vel_cmd.angular.z = 0.0;  

//                     pub.publish(vel_cmd);
//                     ROS_INFO("当前距离: %.3f, 目标: %.2f, 误差: %.3f, PID输出: %.3f", Laser_linear_dis, target_distance, error, output);

//                     // 若距离误差很小则判断完成
//                     if (std::abs(error) < 0.02||(ros::Time::now()-Time_laser).toSec()>=7.0)
//                     {
//                         ROS_INFO("距离控制已完成.");
//                         break;
//                     }
//                 }
//                 check = 3;
//                 ROS_INFO("Going right");
//             }
//             else if (check == 3 )
//             {
//                 is_lidar_update = false;
//                 after_bizhang_x = -20;//避障之后改变检索图像的范围
//                 after_bizhang_y = 40;
//                 check_after_laser = true; 
//                 params.Kp = 0.05;
//                 vel_cmd.linear.x = 0.0;
//                 vel_cmd.linear.y = 0.0;  
//                 vel_cmd.angular.z = 0.0;  
//                 pub.publish(vel_cmd);
//                 break;
//             }
//             //pub.publish(vel_cmd);

//         }
//     }
//     // while(!frame_buffer.empty())
//     // {
//     //     frame_buffer.pop_front();
//     // }

//     move_start_time = ros::Time::now();
//     if(track_type==TRACK_LEFT)
//     {
//         Dis_Bias_Left = 0.0;
//         Dis_Bias_Right = 0.0;
//     }
//     else
//     {
//         Dis_Bias_Left = 0.0;
//         Dis_Bias_Right = 0.0;
//     }
//     // while((ros::Time::now() - move_start_time).toSec()<0.8)
//     // {
//     //     vel_cmd.linear.x = 0.3;
//     //     vel_cmd.linear.y = 0.0;
//     //     vel_cmd.angular.z = 0.0;
//     //     pub.publish(vel_cmd);
//     // }
//     return;
// }