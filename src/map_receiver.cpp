#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <queue>
#include <algorithm>
#include <iostream>
#include <math.h>

double x = 0.0;
double y = 0.0;
double psi = 0.0;

ros::Publisher pub;

struct cell {
    int row;
    int col;
};

nav_msgs::OccupancyGrid occGrid;

bool M[100][100] = { };
int originX = 50;
int originY = 50;
int tileX = 50;
int tileY = 50;
double tileResolution = 0.5;

bool isFree(int xTile, int yTile) {

    if (occGrid.info.resolution == 0) return false;


    // start position of tile
    double xPos = (xTile - originX) * tileResolution;
    double yPos = (yTile - originY) * tileResolution;

    // Find occupancy grid position of tile
    int gridX = (xPos - occGrid.info.origin.position.x) / occGrid.info.resolution;
    int gridY = (yPos - occGrid.info.origin.position.y) / occGrid.info.resolution;

    // Check if all grid cells in the tile are free
    for (int i = 0; i * occGrid.info.resolution < tileResolution; i++) {
        for (int j = 0; j * occGrid.info.resolution < tileResolution; j++) {
            int gridIndex = (gridY+j)*occGrid.info.width+(gridX+i);
            if (occGrid.data[gridIndex] > 10) {
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief BM Boustrophedon motion (BM) algorithm
 */
void BM() {
    /*
    Inputs: The robot’s configuration and the model M of the workspace

    Outputs: Updated version of the robot’s configuration and the model M of the workspace

    Step 1. Check to find the first available direction in the
    priority of north-south-east-west. If all directions are
    blocked, then the critical point has been reached. Break the
    loop.

    Step 2. Move one step along this direction.

    Step 3. Generate the tile s = (x, y, 2r), i.e., the size of the robot’s diameter at the robot’s position.

    Step 4. Add the tile s to the mode M. Go to Step 1.
    */

    // Find tile where robot is located
    tileX = std::floor(x / tileResolution) + originX;
    tileY = std::floor(y / tileResolution) + originY;

    M[tileX][tileY] = true;

    // check if next tile is free for obstacles and not covered already
    int goalX = tileX;
    int goalY = tileY;
    if (isFree(tileX+1, tileY) && !M[tileX+1][tileY]) {
        ROS_WARN_STREAM("Moving to north tile!");
        goalX++;
    } else if (isFree(tileX-1, tileY) && !M[tileX-1][tileY]) {
        ROS_WARN_STREAM("Moving to south tile!");
        goalX--;
    } else if (isFree(tileX, tileY-1) && !M[tileX][tileY-1]) {
        ROS_WARN_STREAM("Moving to east tile!");
        goalY--;
    } else if (isFree(tileX, tileY+1) && !M[tileX][tileY+1]) {
        ROS_WARN_STREAM("Moving to west tile!");
        goalY++;
    } else {
        ROS_WARN_STREAM("Critical point!");
    }

    geometry_msgs::PoseStamped goal;
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = (goalX + 0.5 - originX) * tileResolution;
    goal.pose.position.y = (goalY + 0.5 - originY) * tileResolution;
    goal.pose.position.z = 0.0;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    pub.publish(goal);

}

void mapCallback(const nav_msgs::OccupancyGrid &grid){

    occGrid = grid;
    return;

    int height = grid.info.height;
    int width = grid.info.width;

    // Find grid position of robot
    double resolution = grid.info.resolution;
    double originX = grid.info.origin.position.x;
    double originY = grid.info.origin.position.y;

    // TODO: is origin always the most negative position value?
    // If not, the following will fail. I.e. negative row and col.
    int row = (x - originX) / resolution;
    int col = (y - originY) / resolution;

    // Generate distance transform with BFS
    std::queue<cell> toVisit;
    bool visited[width][height] = { };
    int distanceTransform[width][height] = { };

    cell start = {row, col};
    toVisit.push(start);
    distanceTransform[row][col] = 0;
    visited[row][col] = true;

    while (!toVisit.empty()) {
        cell c = toVisit.front();
        toVisit.pop();

        for (int i = std::max(c.row-1, 0); i < std::min(c.row+2, width); ++i) {
            for (int j = std::max(c.col-1, 0); j < std::min(c.col+2, height); ++j) {
                int gridIndex = j*width+i;
                if (!visited[i][j] && grid.data[gridIndex] >= 0 && grid.data[gridIndex] < 10 ) {
                    cell newCell = {i, j};
                    toVisit.push(newCell);
                    distanceTransform[i][j] = distanceTransform[c.row][c.col] + 1;
                    visited[i][j] = true;
                }
            }
        }
    }

    // Path planning
    std::vector<geometry_msgs::PoseStamped> poses;
    int path[width][height] = { };
    bool cellsVisited[width][height] = { };
    cell c = {row, col};
    path[row][col] = 0;

    int counter = 0;
    while (true) {

        // Find unvisited Neighbouring cell with highest DT
        cell maxCell = {c.row, c.col};
        int maxDT = 0;
        for (int i = std::max(c.row-1, 0); i < std::min(c.row+2, width); i++) {
            for (int j = std::max(c.col-1, 0); j < std::min(c.col+2, height); j++) {
                if (i == c.row && j == c.col) continue;
                if (distanceTransform[i][j] > maxDT && !cellsVisited[i][j]) {
                    maxCell.row = i;
                    maxCell.col = j;
                    maxDT = distanceTransform[i][j];
                }
            }
        }

        // If No Neighbour Cell found then: Mark as Visited and Stop at Goal
        if (maxDT == 0) {
            cellsVisited[c.row][c.col] = true;
            break;
        }

        // If Neighbouring Cell DT <= Current Cell DT then: Mark as Visited and Stop at Goal
        if (maxDT <= distanceTransform[c.row][c.col]) {
            cellsVisited[c.row][c.col] = true;
        }

        // Set Current cell to Neigbouring cell
        c = maxCell;

        // Add to path
        path[c.row][c.col] = ++counter;

        geometry_msgs::PoseStamped newPoint;
        newPoint.pose.position.x = (c.row + 0.5) * resolution + originX;
        newPoint.pose.position.y = (c.col + 0.5) * resolution + originY;
        newPoint.header.stamp = ros::Time(0.0);
        newPoint.header.frame_id = "map";
        poses.push_back(newPoint);
    }

    nav_msgs::Path thePath;
    thePath.header.stamp = ros::Time::now();
    thePath.header.frame_id = "map";
    thePath.poses = poses;

    pub.publish(thePath);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "map_receiver");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("map", 1000, mapCallback);
    pub = node.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1000);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Set current tile to covered
    M[originX][originY] = true;

    ros::Rate rate(10.0);
    while (node.ok()) {

        // get the pose of the robot in the map frame
        geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0.0), ros::Duration(0.0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("Transform from map to base_link not found.");
            continue;
        }
        x = transformStamped.transform.translation.x;
        y = transformStamped.transform.translation.y;
        tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y,
                         transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
        psi = tf::getYaw(q);

        BM();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
