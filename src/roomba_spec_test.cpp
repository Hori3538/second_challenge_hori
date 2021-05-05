#include <roomba_spec_test/roomba_spec_test.h>

RoombaSpecTest::RoombaSpecTest():private_nh("~")
{
    private_nh.getParam("hz", hz);
    private_nh.getParam("direct_control_mode", direct_control_mode);
    private_nh.getParam("input_speed", input_speed);
    private_nh.getParam("intput_yawrate", input_yawrate);

    odometry_sub = nh.subscribe("/roomba/odometry", 10, &RoombaSpecTest::odometry_callback, this);
    roomba_control_pub = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 10);
}

void RoombaSpecTest::odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    previous_odometry = current_odometry;
    current_odometry = *msg;
    if(!odometry_get_flag){previous_odometry = current_odometry;}
    odometry_get_flag = true;
}

void RoombaSpecTest::roomba_ctrl(double speed, double yawrate)
{ 
    roomba_500driver_meiji::RoombaCtrl roomba_control;
    roomba_control.mode = 11;
    roomba_control.cntl.linear.x = speed;
    roomba_control.cntl.angular.z = yawrate;
    roomba_control_pub.publish(roomba_control);

}

std::vector<double> RoombaSpecTest::calc_output()
{
    double dx = current_odometry.pose.pose.position.x - previous_odometry.pose.pose.position.x;
    double dy = current_odometry.pose.pose.position.y - previous_odometry.pose.pose.position.y;
    double current_yaw = tf::getYaw(current_odometry.pose.pose.orientation);
    double previous_yaw = tf::getYaw(previous_odometry.pose.pose.orientation);
    double dyaw = current_yaw - previous_yaw;
    if(dyaw > M_PI){dyaw -= 2*M_PI;}
    if(dyaw < -M_PI){dyaw += 2*M_PI;}

    double dtrans = sqrt(dx*dx + dy*dy);
    double dt = (current_odometry.header.stamp - previous_odometry.header.stamp).toSec();
    double speed = dtrans / dt;
    double yawrate = dyaw / dt;
    std::vector<double> output = {speed, yawrate};

    return output;
}

void RoombaSpecTest::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(direct_control_mode){
            roomba_ctrl(input_speed, input_yawrate);
        }
        if(odometry_get_flag){
            std::vector<double> output = calc_output();
            std::cout.precision(2);
            // std::cout.width(10);
            std::cout << "input: ";
            std::cout.width(5);
            std::cout << current_odometry.twist.twist.linear.x << ", ";
            std::cout.width(5);
            std::cout << current_odometry.twist.twist.angular.z << "  output: ";
            std::cout.width(5);
            std::cout << output[0] << ", " << output[1] << std::endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RoombaSpecTest");
    RoombaSpecTest roomba_spec_test;
    roomba_spec_test.process();

    return 0;
}
