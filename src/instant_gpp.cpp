#include <instant_gpp/instant_gpp.h>

Instant_gpp::Instant_gpp():private_nh("~")
{
    private_nh.param("hz", hz, {1});

    sub_map = nh.subscribe("/map", 100, &Instant_gpp::map_callback, this);
    pub_path = nh.advertise<nav_msgs::Path>("/path", 100);
    path.header.frame_id = "map";
}

void Instant_gpp::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    std::cout << "map_callback" << std::endl;
    map = *msg;
    make_path();
    // pub_path.publish(path);
    std::cout << "end_callback" << std::endl;
    map_get_check = true;
}

void Instant_gpp::make_path()
{
    path.poses.clear();
    path.poses.reserve(2000);
    double res = map.info.resolution;
    geometry_msgs::PoseStamped pose;
    for(int i=0; i<5; i++){
        std::cout << "i: " << i << std::endl;
        double start_x = goal_points[i].first;
        double start_y = goal_points[i].second;
        double goal_x = goal_points[i+1].first;
        double goal_y = goal_points[i+1].second;
        if(goal_x != start_x){
            pose.pose.position.y = start_y;
            if(goal_x > start_x){
                for(double x=start_x; x<goal_x; x+=res){
                    pose.pose.position.x = x;
                    path.poses.push_back(pose);
                    // std::cout << "c" << std::endl;
                }
            }
            else{
                for(double x=start_x; x>goal_x; x-=res){
                    pose.pose.position.x = x;
                    path.poses.push_back(pose);
                    // std::cout << "c" << std::endl;
                }
            }
        }
        else{
            pose.pose.position.x = start_x;
            if(goal_y > start_y){
                for(double y=start_y; y<goal_y; y+=res){
                    pose.pose.position.y = y;
                    path.poses.push_back(pose);
                }
            }
            else{
                for(double y=start_y; y>goal_y; y-=res){
                    pose.pose.position.y = y;
                    path.poses.push_back(pose);
                }
            }
        }
    }
}

void Instant_gpp::process()
{
    ros::Rate rate(hz);
    while(ros::ok()){
        if(map_get_check){
        pub_path.publish(path);
        }
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Instant_gpp");
    Instant_gpp instant_gpp;
    instant_gpp.process();
    return 0;
}
