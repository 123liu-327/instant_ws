#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseLocalPlanner)

double Kp = 0.5;
double Kd = 0.02;
double Ki = 0.02;

double angular_error = 0.0;
double last_error = 0.0;
double error_sum = 0.0; //误差累积
double error_diff = 0.0; //误差变化率
double output = 0.0;  //PID输出值


namespace my_planner {
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL, "");
    }
    MyPlanner::~MyPlanner()
    {}
    
    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("规划器启动");
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;
        ros::NodeHandle nh;
        map_image_pub_ = nh.advertise<sensor_msgs::Image>("/map_image", 1);
    }
         
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    int prev_target_index_ = -1;  // 追踪上一个目标点，用于重置积分
    bool pose_adjusting_;
    bool goal_reached_;
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_ = 0;
        prev_target_index_ = -1;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached_ = false;
        error_sum = 0.0;  // 重置全局积分
        last_error = 0.0;
        return true;
    }
           

    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        //代价地图数据获取
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        unsigned char* costmap_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        //使用openCV绘制
        cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128, 128, 128));
        for(unsigned int y=0;y<size_y;y++){
            for(unsigned int x=0;x<size_x;x++){
                int map_index = y*size_x+x;
                unsigned char cost = costmap_data[map_index];//从代价地图获取数据
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(map_index);
                if(cost == 0){
                    pixel = cv::Vec3b(128, 128, 128);
                }
                else if(cost == 254){
                    pixel = cv::Vec3b(0, 0, 0);
                }
                else if(cost == 253){
                    pixel = cv::Vec3b(255, 255, 0);
                }
                else{
                    unsigned char blue = 255 - cost;
                    unsigned char red = cost;
                    pixel = cv::Vec3b(blue, 0, red);
                }
            }
        }
        //导航路线绘制
        for(int i=0;i<global_plan_.size();i++){
            geometry_msgs::PoseStamped pose_odom;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("odom", global_plan_[i], pose_odom); 
            double odom_x = pose_odom.pose.position.x;
            double odom_y = pose_odom.pose.position.y;

            double origin_x = costmap->getOriginX();
            double origin_y = costmap->getOriginY();
            double local_x = odom_x - origin_x;
            double local_y = odom_y - origin_y;
            int x = local_x/costmap->getResolution();
            int y = local_y/costmap->getResolution();
            cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(255, 0, 255));
            
            if(i >= target_index_ && i < target_index_+10){
                cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(0, 255, 255));
                int map_index = y*size_x+x;
                unsigned char cost = costmap_data[map_index];
                if(cost >= 253){
                    return false;
                }
            }
        }

        //显示机器人位置
        map_image.at<cv::Vec3b>(size_y/2, size_x/2) = cv::Vec3b(0, 255, 0);
        //翻转costmap
        
        cv::Mat flipped_image(size_x, size_y, CV_8UC3, cv::Scalar(128, 128, 128));
        for (unsigned int y=0;y<size_y;++y){
            for (unsigned int x=0;x<size_x;++x){
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
                flipped_image.at<cv::Vec3b>((size_x-1-x), (size_y-1-y)) = pixel;
            }
        }
        map_image = flipped_image;

        if (!map_image.empty()) {
            cv_bridge::CvImage cv_map_img;
            cv_map_img.header.stamp = ros::Time::now();
            cv_map_img.header.frame_id = costmap_ros_->getGlobalFrameID();
            cv_map_img.encoding = "bgr8";
            cv_map_img.image = map_image;
            map_image_pub_.publish(cv_map_img.toImageMsg());
        }

        //调整位姿
        int final_index = global_plan_.size()-1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose("base_link", global_plan_[final_index], pose_final);
        if(pose_adjusting_==false){
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx*dx+dy*dy);
            if (dist < 0.005){
                pose_adjusting_ = true;
            }
        }

        if (pose_adjusting_==true)
        {
            cmd_vel.linear.x = 0.0;
            cmd_vel.linear.y = 0.0;
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_WARN("调整角度%.3f", final_yaw);

            last_error = 0;
            double integral = 0;

            const double angle_gain = 1.0;   // 角度→速度增益
            const double slow_zone = 0.2;    // rad (~11°): 提前开始减速
            const double min_speed  = 0.08;  // 最低角速度，精细调整
            const double tolerance  = 0.015; // rad (~0.86°): 到位容差，90°就是90°

            double abs_yaw = fabs(final_yaw);
            double raw_speed = abs_yaw * angle_gain;

            if (abs_yaw < tolerance) {
                goal_reached_ = true;
                ROS_WARN("到达目标点");
                cmd_vel.angular.z = 0.0;
                integral = 0;
                last_error = 0;
            } else if (abs_yaw > slow_zone) {
                // 远离目标：全速，但封顶
                if (raw_speed > 1.2) raw_speed = 1.2;
                cmd_vel.angular.z = (final_yaw > 0) ? raw_speed : -raw_speed;
            } else {
                // 接近目标：线性减速，从 angle_gain*slow_zone 降到 min_speed
                double speed = min_speed + (raw_speed - min_speed) * (abs_yaw / slow_zone);
                cmd_vel.angular.z = (final_yaw > 0) ? speed : -speed;
            }
            return true;
        }
        
        

        //规划路线追踪
        geometry_msgs::PoseStamped target_pose;
        //坐标转化，所有的坐标必须专程机器人坐标系
        for(int i=target_index_;i<global_plan_.size();i++){
            geometry_msgs::PoseStamped pose_base;
            geometry_msgs::PoseStamped pose_in = global_plan_[i]; // 创建副本避免修改原数据
            pose_in.header.stamp = ros::Time(0); 
            try {
                // 确保从 map 到 base_link 的转换（假设全局路径的坐标系是 map）
                tf_listener_->transformPose("base_link", pose_in, pose_base);
            } 
            catch (tf::TransformException &ex) {// 避免使用无效的坐标数据
                ROS_ERROR("TF转换失败: %s", ex.what());
                return false; 
            }
           
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx*dx+dy*dy);
            if (dist > 0.15){
                target_pose = pose_base;
                target_index_ = i;
                ROS_WARN("选择第%d个目标点，距离=%.2f",target_index_, dist);
                break;
            }

            if(i == global_plan_.size()-1) target_pose = pose_base;   
        }

        //计算追踪目标点的速度
        const double linear_gain = 3.0;
        cmd_vel.linear.x = target_pose.pose.position.x * linear_gain;
        cmd_vel.linear.y = target_pose.pose.position.y * linear_gain;

        // 切换到新航点时重置积分，防止跨航点累积
        if (target_index_ != prev_target_index_) {
            error_sum = 0.0;
            last_error = 0.0;
            prev_target_index_ = target_index_;
        }

        angular_error = target_pose.pose.position.y;
        error_sum += angular_error;
        // 积分抗饱和：限制在 ±0.5 范围内
        if (error_sum > 0.5)  error_sum = 0.5;
        if (error_sum < -0.5) error_sum = -0.5;
        error_diff = angular_error - last_error;
        output = Kp*angular_error + Kd*error_diff + Ki*error_sum;
        cmd_vel.angular.z = output;
        last_error = angular_error;
        
        cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
        for(int i=0;i<global_plan_.size();i++){
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            //全局路径坐标系转化机器人坐标系
            tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
            int cv_x = 300 - pose_base.pose.position.y*100;
            int cv_y = 300 - pose_base.pose.position.x*100;
            cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255));
        }

        cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
        cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0), 1);
        cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0), 1);
        
        cv::waitKey(1);
        return true;
    }
        
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
}