#ifndef LOCAL_PATH_PLANNER
#define LOCAL_PATH_PLANNER

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


struct State
{
    double x;
    double y;
    double yaw;
    double speed;
    double yawrate;
};

class LocalPathPlanner
{
    public:
        LocalPathPlanner();
        void process();
    private:
        void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void obstacle_poses_callback(const geometry_msgs::PoseArray::ConstPtr &msg);

        double adjust_yaw(double yaw);
        void roomba_move(State &state, double speed, double yawrate);
        std::vector<double> calc_dynamic_window();
        std::vector<State> calc_trajectory(double speed, double yawrate);
        double calc_heading_score(std::vector<State> &trajectory);
        double calc_dist_score(std::vector<State> &trajectory);
        std::vector<double> decide_input();
        void roomba_ctrl(double speed, double yawrate);
        void visualize_trajectory(std::vector<State> &trajectory, ros::Publisher &publisher);

        int hz;
        double goal_tolerance;

        double roomba_radius;
        double max_speed;
        double min_speed;
        double max_yawrate;
        double max_accel;
        double max_dyawrate;

        double speed_reso;
        double yawrate_reso;
        double dt;
        double predict_time;
        double heading_score_gain;
        double velocity_score_gain;
        double dist_score_gain;

        const int INF = 1e9;
        bool local_goal_get_check = false;
        bool obstacle_poses_get_check = false;
        std::vector<double> previous_input;
        std::vector<std::vector<State>> trajectories;
        std::vector<State> best_trajectory;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber local_goal_sub;
        ros::Subscriber obstacle_poses_sub;
        ros::Publisher roomba_control_pub;
        ros::Publisher best_local_path_pub;
        ros::Publisher local_paths_pub;
        ros::Publisher local_goal_point_pub;

        geometry_msgs::PoseStamped local_goal;
        geometry_msgs::PointStamped local_goal_point;
        geometry_msgs::PoseArray obstacle_poses;

        tf2_ros::Buffer tf_buffer;
};

#endif
