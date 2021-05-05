#include <instant_gpp/instant_gpp.h>

InstantGpp::InstantGpp():private_nh("~")
{
    private_nh.getParam("hz", hz);

    map_sub = nh.subscribe("/map", 1, &InstantGpp::map_callback, this);
    global_path_pub = nh.advertise<nav_msgs::Path>("/global_path", 1);
    global_path.header.frame_id = "map";
}

void InstantGpp::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
    make_global_path();
    map_get_check = true;
}

void InstantGpp::make_global_path()
{
    global_path.poses.clear();
    global_path.poses.reserve(2000);
    double res = map.info.resolution;
    geometry_msgs::PoseStamped pose;
    for(int i=0; i<int(goal_points.size()); i++){
        double start_x = goal_points[i].first;
        double start_y = goal_points[i].second;
        double goal_x = goal_points[i+1].first;
        double goal_y = goal_points[i+1].second;
        if(goal_x != start_x){
            pose.pose.position.y = start_y;
            if(goal_x > start_x){
                for(double x=start_x; x<goal_x; x+=res){
                    pose.pose.position.x = x;
                    global_path.poses.push_back(pose);
                }
            }
            else{
                for(double x=start_x; x>goal_x; x-=res){
                    pose.pose.position.x = x;
                    global_path.poses.push_back(pose);
                }
            }
        }
        else{
            pose.pose.position.x = start_x;
            if(goal_y > start_y){
                for(double y=start_y; y<goal_y; y+=res){
                    pose.pose.position.y = y;
                    global_path.poses.push_back(pose);
                }
            }
            else{
                for(double y=start_y; y>goal_y; y-=res){
                    pose.pose.position.y = y;
                    global_path.poses.push_back(pose);
                }
            }
        }
    }
}

void InstantGpp::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok()){
        if(map_get_check){
        global_path_pub.publish(global_path);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "instant_gpp");
    InstantGpp instant_gpp;
    instant_gpp.process();

    return 0;
}
