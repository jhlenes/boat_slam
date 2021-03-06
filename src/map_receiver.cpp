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
#include <vector>

double x = 0.0;
double y = 0.0;
double psi = 0.0;

ros::Publisher pub;
ros::Publisher pub2;
nav_msgs::Path coveredPath;


struct cell {
    int row;
    int col;
};

nav_msgs::OccupancyGrid occGrid;

const int UNKNOWN = 0;
const int FREE = 1;
const int COVERED = 2;
const int BLOCKED = 3;

struct tile {
    int x;
    int y;
};

const int TILE_SIZE = 100;
int M[TILE_SIZE][TILE_SIZE] = {};
std::vector<tile> BP;
int originX = 50;
int originY = 50;
int tileX = 50;
int tileY = 50;
double tileResolution = 0.50;

bool isBacktrackingPoint(int i, int j) {
    // b(s1,s8) or b(s1,s2)
    bool eastFree = M[i][j-1] == FREE && (M[i+1][j-1] >= COVERED || M[i-1][j-1] >= COVERED);

    // b(s5,s6) or b(s5,s4)
    bool westFree = M[i][j+1] == FREE && (M[i+1][j+1] >= COVERED || M[i-1][j+1] >= COVERED);

    // b(s7,s6) or b(s7,s8)
    bool southFree = M[i-1][j] == FREE && (M[i-1][j+1] >= COVERED || M[i-1][j-1] >= COVERED);

    return eastFree || westFree || southFree;
}

void locateBestBacktrackingPoint(int &goalX, int &goalY, int tileX, int tileY) {

    // find points
    BP.clear();
    int closestPoint = 0;
    double minDistance = -1.0;
    for (int i = 1; i < TILE_SIZE-1; i++) {
        for (int j = 1; j < TILE_SIZE-1; j++) {
            if (M[i][j] != COVERED) {
                continue;
            }
            if (isBacktrackingPoint(i, j)) {
                tile point = {i, j};
                BP.push_back(point);

                double dist = std::sqrt(std::pow(i - tileX, 2) + std::pow(j - tileY, 2));
                if (dist < minDistance || minDistance < 0) {
                    closestPoint = BP.size() - 1;
                    minDistance = dist;
                }
            }
        }
    }

    // find the nearest point. TODO: change to shortest path on covered tiles
    if (BP.size() > 0) {
        tile best = BP.at(closestPoint);
        goalX = best.x;
        goalY = best.y;
    }
}

bool isFree(int xTile, int yTile) {

    if (occGrid.info.resolution == 0) return false;

    // Find start position of tile
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


    // check if next tile is free for obstacles and not covered already
    static bool hasGoal = false;
    static int goalX = tileX;
    static int goalY = tileY;
    if (goalX == tileX && goalY == tileY) {
        hasGoal = false;
        M[tileX][tileY] = COVERED;
    }

    if (M[tileX+1][tileY] != COVERED) {
        if (isFree(tileX+1, tileY)) {
            if (!hasGoal) {
                ROS_WARN_STREAM("Moving to north tile!");
                goalX++;
                hasGoal = true;
            } else {
                M[tileX+1][tileY] = FREE;
            }
        } else {
            M[tileX+1][tileY] = BLOCKED;
        }
    }
    if (M[tileX-1][tileY] != COVERED) {
        if (isFree(tileX-1, tileY)) {
            if (!hasGoal) {
                ROS_WARN_STREAM("Moving to south tile!");
                goalX--;
                hasGoal = true;
            } else {
                M[tileX-1][tileY] = FREE;
            }
        } else {
            M[tileX-1][tileY] = BLOCKED;
        }
    }
    if (M[tileX][tileY-1] != COVERED) {
        if (isFree(tileX, tileY-1)) {
            if (!hasGoal) {
                ROS_WARN_STREAM("Moving to east tile!");
                goalY--;
                hasGoal = true;
            } else {
                M[tileX][tileY-1] = FREE;
            }
        } else {
            M[tileX][tileY-1] = BLOCKED;
        }
    }
    if (M[tileX][tileY+1] != COVERED) {
        if (isFree(tileX, tileY+1)) {
            if (!hasGoal) {
                ROS_WARN_STREAM("Moving to west tile!");
                goalY++;
                hasGoal = true;
            } else {
                M[tileX][tileY+1] = FREE;
            }
        } else {
            M[tileX][tileY+1] = BLOCKED;
        }
    }
    if (!hasGoal) {
        ROS_WARN_STREAM("Critical point!");
        locateBestBacktrackingPoint(goalX, goalY, tileX, tileY);
        hasGoal = true;
    }

    // publish goal tile and covered path
    geometry_msgs::PoseStamped goal;

    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = "map";
    goal.pose.position.x = (goalX + 0.5 - originX) * tileResolution;
    goal.pose.position.y = (goalY + 0.5 - originY) * tileResolution;
    goal.pose.position.z = 0.0;

    double psi = std::atan2(goalY-tileY, goalX-tileX);
    tf::Quaternion q = tf::createQuaternionFromYaw(psi);

    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    pub.publish(goal);

    coveredPath.header.stamp = ros::Time::now();
    coveredPath.header.frame_id = "map";
    coveredPath.poses.push_back(goal);

    pub2.publish(coveredPath);
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
    pub2 = node.advertise<nav_msgs::Path>("mypath", 1000);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Set current tile to covered
    M[originX][originY] = COVERED;

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
