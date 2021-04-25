#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class Local_goal_creator
{
    public:
        Local_goal_creator();
        void process();
    private:
        void path_callback(const nav_msgs::Path::ConstPtr &msg);
        void estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void select_local_goal();

        int hz;
        double local_goal_dist;

        int goal_index = 0;
        bool path_get_check = false;
        bool estimated_pose_check = false;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber path_sub;
        ros::Subscriber estimated_pose_sub;
        ros::Publisher local_goal_pub;

        nav_msgs::Path path;
        geometry_msgs::PoseStamped estimated_pose;
        geometry_msgs::PoseStamped local_goal;
};
#endif
