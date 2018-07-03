#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <queue>
#include <string>
#include <algorithm>

double x = 0.0;
double y = 0.0;

struct cell {
    int row;
    int col;
};

void mapCallback(const nav_msgs::OccupancyGrid &grid){

    uint32_t height = grid.info.height;
    uint32_t width = grid.info.width;

    // Find grid position of robot
    float resolution = grid.info.resolution;
    double originX = grid.info.origin.position.x;
    double originY = grid.info.origin.position.x;
    ROS_WARN_STREAM("X: " << originX);
    int row = (x - originX) / resolution;
    int col = (y - originY) / resolution;

    // generate distance transform with BFS
    std::queue<cell> toVisit;
    bool visited[height][width];
    cell start = {row, col};
    toVisit.push(start);
    int distanceTransform[height][width];
    distanceTransform[row][col] = 0;
    visited[row][col] = true;

    while (!toVisit.empty()) {
        cell c = toVisit.pop();

        for (int i = std::max(c.row-1, 0); i < std::min(c.row+1, height-1); i++) {
            for (int j = std::max(c.col-1, 0); j < std::min(c.col+1, width-1); j++) {
                if (!visited[i][j]) {
                    cell newCell = {i, j};
                    toVisit.push(newCell);
                    distanceTransform[i][j] == distanceTransform[c.row][c.col]+1;
                    visited[i][j] = true;
                }
            }
        }
    }

    for (size_t row = 0; row < height; ++row) {
        for (size_t col = 0; col < width; ++col) {
            int8_t prob = grid.data[row*width + col];
        }
    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_receiver");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("map", 1000, mapCallback);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(10.0);
    while (node.ok()) {

        // get the position
        geometry_msgs::TransformStamped transformStamped;
        try{
          transformStamped = tfBuffer.lookupTransform("base_link", "map", ros::Time(0.0), ros::Duration(0.0));
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("Transform from map to base_link not found.");
          continue;
        }

        x = transformStamped.transform.translation.x;
        y = transformStamped.transform.translation.y;

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
