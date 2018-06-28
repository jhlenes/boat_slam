#include <ros/ros.h>
#include <stdio.h>

#include <nav_msgs/OccupancyGrid.h>

void mapCallback(const nav_msgs::OccupancyGrid &map){

    ROS_INFO("Map received!");
    uint32_t height = map.info.height;
    uint32_t width = map.info.width;
    ROS_INFO_STREAM("Height: " << height << "\tWidth: " << width);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_receiver");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("map", 1000, mapCallback);

    ros::spin();
    return 0;
}
