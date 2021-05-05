#ifndef INSTANT_GPP_H
#define INSTANT_GPP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

typedef std::pair<double, double> P;

class InstantGpp
{
    public:
        InstantGpp();
        void process();
    private:
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
        void make_global_path();
        int hz;
        bool map_get_check = false; 
        std::vector<P> goal_points = {P(0,0),P(10.5,0),P(10.5,14.25),P(-22.5,14.25),P(-22.5,0),P(0,0)};

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber map_sub;
        ros::Publisher global_path_pub;

        nav_msgs::OccupancyGrid map;
        nav_msgs::Path global_path;
};

#endif
