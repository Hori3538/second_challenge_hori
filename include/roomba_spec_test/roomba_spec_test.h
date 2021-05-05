#ifndef ROOMBA_SPEC_TEST
#define ROOMBA_SPEC_TEST

#include <geometry_msgs/Twist.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

class RoombaSpecTest
{
    public:
        RoombaSpecTest();
        void process();
    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void roomba_ctrl(double speed, double yawrate);
        std::vector<double> calc_output();

        int hz;
        bool direct_control_mode;
        double input_speed;
        double input_yawrate;

        bool odometry_get_flag = false;

        ros::NodeHandle private_nh;
        ros::NodeHandle nh;
        ros::Subscriber odometry_sub;
        ros::Publisher roomba_control_pub;

        nav_msgs::Odometry current_odometry;
        nav_msgs::Odometry previous_odometry;
};

#endif
