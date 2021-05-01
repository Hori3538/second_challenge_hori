#include <local_path_planner/local_path_planner.h>

LocalPathPlanner::LocalPathPlanner():private_nh("~")
{
    private_nh.getParam("hz", hz);
    private_nh.getParam("roomba_radius", roomba_radius);
    private_nh.getParam("max_speed", max_speed);
    private_nh.getParam("min_speed", min_speed);
    private_nh.getParam("max_yawrate", max_yawrate);
    private_nh.getParam("max_accel", max_accel);
    private_nh.getParam("max_dyawrate", max_dyawrate);
    private_nh.getParam("speed_reso", speed_reso);
    private_nh.getParam("yawrate_reso", yawrate_reso);
    private_nh.getParam("dt", dt);
    private_nh.getParam("predict_time", predict_time);
    private_nh.getParam("heading_score_gain", heading_score_gain);
    private_nh.getParam("velocity_score_gain", velocity_score_gain);
    private_nh.getParam("dist_score_gain", dist_score_gain);


    local_goal_sub = nh.subscribe("/local_goal", 1, &LocalPathPlanner::local_goal_callback, this);
    obstacle_poses_sub = nh.subscribe("/obstacle_poses", 1, &LocalPathPlanner::obstacle_poses_callback, this);

    roomba_control_pub = nh.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    local_paths_pub = nh.advertise<nav_msgs::Path>("local_paths", 1);
    best_local_path_pub = nh.advertise<nav_msgs::Path>("best_local_path", 1);
    local_goal_point_pub = nh.advertise<geometry_msgs::PointStamped>("local_goal_point", 1);

    local_goal_point.header.frame_id = "base_link";
    previous_input = {0.0, 0.0};
}

void LocalPathPlanner::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_goal = *msg;
    local_goal.pose.orientation.w = 1;

    try{
        geometry_msgs::TransformStamped transform;
        transform = tf_buffer.lookupTransform("base_link", "map", ros::Time(0));
        tf2::doTransform(local_goal, local_goal, transform);

        local_goal_point.point.x = local_goal.pose.position.x;
        local_goal_point.point.y = local_goal.pose.position.y;
        local_goal_get_check = true;
    }
    catch(tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());

        local_goal_get_check = false;
    }
}

void LocalPathPlanner::obstacle_poses_callback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    obstacle_poses = *msg;
    obstacle_poses_get_check = true;
}

double LocalPathPlanner::adjust_yaw(double yaw)
{
    if(yaw > M_PI){yaw -= 2*M_PI;}
    if(yaw < -M_PI){yaw += 2*M_PI;}

    return yaw;
}

void LocalPathPlanner::roomba_move(State &state, double speed, double yawrate)
{
    state.yaw += yawrate * dt;
    state.yaw = adjust_yaw(state.yaw);

    state.x += speed * std::cos(state.yaw) * dt;
    state.y += speed * std::sin(state.yaw) * dt;

    state.speed = speed;
    state.yawrate = yawrate;
}

std::vector<double> LocalPathPlanner::calc_dynamic_window()
{
    std::vector<double> Vs = {min_speed, max_speed, -max_yawrate, max_yawrate};
    std::vector<double> Vd(4);
    std::vector<double> dynamic_window(4);

    Vd = {previous_input[0] - max_accel * dt, previous_input[0] + max_accel * dt,
          previous_input[1] - max_dyawrate * dt, previous_input[1] + max_dyawrate * dt};

    dynamic_window = {std::max(Vs[0], Vd[0]), std::min(Vs[1], Vd[1]),
                      std::max(Vs[2], Vd[2]), std::min(Vs[3], Vd[3])};

    return dynamic_window;
}

std::vector<State> LocalPathPlanner::calc_trajectory(double speed, double yawrate)
{
    State state = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<State> trajectory;
    for(double t=0.0; t<=predict_time; t+=dt){
        roomba_move(state, speed, yawrate);
        trajectory.push_back(state);
    }

    return trajectory;
}

double LocalPathPlanner::calc_heading_score(std::vector<State> &trajectory)
{
    State last_state = trajectory.back();
    double angle_to_goal = std::atan2(local_goal_point.point.y - last_state.y,
                                      local_goal_point.point.x - last_state.x);
    angle_to_goal -= last_state.yaw;
    double heading_score = M_PI - std::abs(adjust_yaw(angle_to_goal));

    return heading_score;
}

double LocalPathPlanner::calc_dist_score(std::vector<State> &trajectory)
{
    double min_dist = INF;
    for(auto& state : trajectory){
        for(auto& obstacle_pose : obstacle_poses.poses){
            double obstacle_x = obstacle_pose.position.x;
            double obstacle_y = obstacle_pose.position.y;
            double dx = obstacle_x - state.x;
            double dy = obstacle_y - state.y;

            double dist = std::sqrt(dx * dx + dy * dy);
            if(dist <= roomba_radius){
                return -INF;
            }
            if(dist < min_dist){
                min_dist = dist;
            }
        }
    }

    return min_dist;
}

std::vector<double> LocalPathPlanner::decide_input()
{
    std::vector<double> input{0.0, 0.0};
    std::vector<double> dynamic_window = calc_dynamic_window();
    double best_score = 0;
    trajectories.clear();

    for(double speed=dynamic_window[0]; speed<=dynamic_window[1]; speed+=speed_reso){
        for(double yawrate=dynamic_window[2]; yawrate<=dynamic_window[3]; yawrate+=yawrate_reso){
            std::vector<State> trajectory = calc_trajectory(speed, yawrate);
            trajectories.push_back(trajectory);

            double heading_score = heading_score_gain * calc_heading_score(trajectory);
            double dist_score = dist_score_gain * calc_dist_score(trajectory);
            double velocity_score = velocity_score_gain * speed;
            double final_score = heading_score + dist_score + velocity_score;

            if(final_score > best_score){
                best_score = final_score;
                input = {speed, yawrate};
                best_trajectory = trajectory;
            }
        }
    }
    previous_input = input;

    return input;
}

void LocalPathPlanner::roomba_ctrl(double speed, double yawrate)
{
    roomba_500driver_meiji::RoombaCtrl roomba_control;
    roomba_control.mode = 11;
    roomba_control.cntl.linear.x = speed;
    roomba_control.cntl.angular.z = yawrate;
    roomba_control_pub.publish(roomba_control);
}

void LocalPathPlanner::visualize_trajectory(std::vector<State> &trajectory, ros::Publisher &publisher)
{
    nav_msgs::Path local_path;
    local_path.header.frame_id = "base_link";
    local_path.header.stamp = ros::Time::now();

    for(auto& state : trajectory){
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        local_path.poses.push_back(pose);
    }
    publisher.publish(local_path);
}

void LocalPathPlanner::process()
{
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(local_goal_get_check && obstacle_poses_get_check){
            std::vector<double> input = decide_input();
            roomba_ctrl(input[0], input[1]);
            for(auto& trajectory : trajectories){
                visualize_trajectory(trajectory, local_paths_pub);
            }
            visualize_trajectory(best_trajectory, best_local_path_pub);
            local_goal_point_pub.publish(local_goal_point);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_path_planner");
    LocalPathPlanner local_path_planner;
    local_path_planner.process();

    return 0;
}
