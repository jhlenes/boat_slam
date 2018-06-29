#include <ros/ros.h>
#include <stdio.h>

#include <nav_msgs/OccupancyGrid.h>

void mapCallback(const nav_msgs::OccupancyGrid &map){

    ROS_INFO("Map received!");
    const unsigned int height = map.info.height;
    const unsigned int width = map.info.width;
    ROS_INFO_STREAM("Height: " << height << "\tWidth: " << width);

    for (size_t row = 0; row < height; ++row) {
        for (size_t col = 0; col < width; ++col) {
            const signed char a = map.data[row*width + col];
            std::cout << int(a) << " ";
        }
        std::cout << "\n";
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_receiver");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("map", 1000, mapCallback);

    ros::spin();
    return 0;
}
