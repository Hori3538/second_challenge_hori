#include <local_goal_creator/local_goal_creator.h>

Local_goal_creator::Local_goal_creator():private_nh("~")
{
    private_nh.getParam("hz", hz);
    private_nh.getParam("local_goal_dist", local_goal_dist);
    path_sub = nh.subscribe("/path", 1, &Local_goal_creator::path_callback, this);
    estimated_pose_sub = nh.subscribe("/mcl_pose", 1, &Local_goal_creator::estimated_pose_callback, this);

    local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
}

void Local_goal_creator::path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    path = *msg;
    path_get_check = true;
}

void Local_goal_creator::estimated_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    estimated_pose = *msg;
    estimated_pose_check = true;
    if(path_get_check){
        select_local_goal();
    }
}

void Local_goal_creator::select_local_goal()
{
    double dist = sqrt(pow(estimated_pose.pose.position.x - path.poses[goal_index].pose.position.x, 2) + pow(estimated_pose.pose.position.y - path.poses[goal_index].pose.position.y, 2));
    while(dist < local_goal_dist && goal_index+1 < int(path.poses.size())){
        dist += 0.05;
        goal_index += 1;
    }
    local_goal = path.poses[goal_index];
    local_goal.header.frame_id = "map";
    local_goal.header.stamp = ros::Time::now();
}

void Local_goal_creator::process()
{
    ros::Rate rate(hz);
    while(ros::ok()){
        if(path_get_check && estimated_pose_check){
            local_goal_pub.publish(local_goal);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_goal_creator");
    Local_goal_creator local_goal_creator;
    local_goal_creator.process();

    return 0;
}
