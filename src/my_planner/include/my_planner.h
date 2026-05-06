#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

namespace my_planner
{
    class MyPlanner : public nav_core::BaseLocalPlanner
    {
        public:
            ros::Publisher map_image_pub_;
            //ros::Publisher plan_image_pub_;
            MyPlanner();
            ~MyPlanner();

            //初始化函数--Ubuntu20.04
            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
            //接受全局规划器路线
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            //对cmd_vel进行规划
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
            //提交导航结果
            bool isGoalReached();
    };
}

#endif