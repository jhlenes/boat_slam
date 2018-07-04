#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <queue>
#include <algorithm>
#include <vector>
#include <iostream>

double x = 0.0;
double y = 0.0;

struct cell {
    int row;
    int col;
};

void mapCallback(const nav_msgs::OccupancyGrid &grid){

    ROS_WARN("MAP RECEIVED!");

    int height = grid.info.height;
    int width = grid.info.width;

    ROS_WARN_STREAM("Size: " << height << ", " << width);

    // Find grid position of robot
    double resolution = grid.info.resolution;
    double originX = grid.info.origin.position.x;
    double originY = grid.info.origin.position.x;
    // TODO: is origin always the most negative position value?
    // If not, the following will fail. I.e. negative row and col.
    int row = (x - originX) / resolution;
    int col = (y - originY) / resolution;
    ROS_WARN_STREAM("Position: " << row << ", " << col);

    // Generate distance transform with BFS
    std::queue<cell> toVisit;
    bool visited[height][width] = { };
    int distanceTransform[height][width] = { };

    cell start = {row, col};
    toVisit.push(start);
    distanceTransform[row][col] = 0;
    visited[row][col] = true;

    while (!toVisit.empty()) {
        cell c = toVisit.front();
        toVisit.pop();

        for (int i = std::max(c.row-1, 0); i < std::min(c.row+2, height); ++i) {
            for (int j = std::max(c.col-1, 0); j < std::min(c.col+2, width); ++j) {
                if (!visited[i][j] && grid.data.at(i*width+col) >= 0 && grid.data.at(i*width+col) < 50 ) {
                    cell newCell = {i, j};
                    toVisit.push(newCell);
                    distanceTransform[i][j] = distanceTransform[c.row][c.col] + 1;
                    visited[i][j] = true;
                }
            }
        }
    }

    /*
    for (std::size_t i = 0; i < height; i++) {
        for (std::size_t j = 0; j < width; j++) {
            std::cout << distanceTransform[i][j] << ", ";
        }
        std::cout << std::endl;
    }
    */


    // Path planning
    std::vector<geometry_msgs::PoseStamped> poses;
    int path[height][width] = { };
    bool cellsVisited[height][width] = { };
    cell c = {row, col};
    path.push_back(c);

    while (true) {

        // Find unvisited Neighbouring cell with highest DT
        cell maxCell = {c.row, c.col};
        int maxDT = 0;
        for (int i = std::max(c.row-1, 0); i < std::min(c.row+2, height); i++) {
            for (int j = std::max(c.col-1, 0); j < std::min(c.col+2, width); j++) {
                if (distanceTransform[i][j] > maxDT) {
                    maxCell.row = i;
                    maxCell.col = j;
                    maxDT = distanceTransform[i][j];
                }
            }
        }

        // If No Neighbour Cell found then: Mark as Visited and Stop at Goal
        if (maxDT == 0) {
            cellsVisited[c.row][c.col] = true;
        }

        // If Neighbouring Cell DT <= Current Cell DT then: Mark as Visited and Stop at Goal
        if (maxDT <= distanceTransform[c.row][c.col]) {
            cellsVisited[c.row][c.col] = true;
        }

        // Set Current cell to Neigbouring cell
        c = maxCell;

        path[c.row][c.col] = 1;
        path.push_back(c);
        if (path.size() > 10) {
            break;
        }
    }

    for (std::size_t i = 0; i < 10; i++) {
        cell myCell = path.at(i);
        std::cout << "[" << myCell.row << ", " << myCell.col << "], ";
    }
    std::cout << std::endl;

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
