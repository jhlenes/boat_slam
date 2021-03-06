#include <pluginlib/class_list_macros.h>
#include "global_planner.h"
#include <nav_msgs/Path.h>

#include <queue>
#include <stack>
#include <iostream>

#define DEFAULT_MIN_DIST_FROM_ROBOT 0.10

struct cell {
    int x;
    int y;
};

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner, nav_core::BaseGlobalPlanner)

namespace global_planner {

    GlobalPlanner::GlobalPlanner ()
    : costmap_ros_(NULL), initialized_(false){

    }

    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_ros_(NULL), initialized_(false){
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);
            private_nh.param("step_size", step_size_, costmap_->getResolution());
            private_nh.param("min_dist_from_robot", min_dist_from_robot_, DEFAULT_MIN_DIST_FROM_ROBOT);
            world_model_ = new base_local_planner::CostmapModel(*costmap_);

            initialized_ = true;
        } else {
            ROS_WARN("This planner has already been initialized... doing nothing");
        }
    }

    //we need to take the footprint of the robot into account when we calculate cost to obstacles
    double GlobalPlanner::footprintCost(double x_i, double y_i, double theta_i){
        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return -1.0;
        }

        std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
        //if we have no footprint... do nothing
        if(footprint.size() < 3) {
            return -1.0;
        }

        //check if the footprint is legal
        double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
        return footprint_cost;
    }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        if(!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        plan.clear();
        costmap_ = costmap_ros_->getCostmap();

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
            costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        tf::Stamped<tf::Pose> goal_tf;
        tf::Stamped<tf::Pose> start_tf;

        poseStampedMsgToTF(goal,goal_tf);
        poseStampedMsgToTF(start,start_tf);

        double useless_pitch, useless_roll, goal_yaw, start_yaw;
        start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
        goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

        /*
         * 1) Use costmap2d to divide the map into occupied and free cells
         * 2) Identify the closest cells to the desired start and end pose
         * 3) Use a wavefront algorithm to assign the distance transform value at each cell
         * 4) Iterate through these cells in a path of slowest decent as described in the paper
        */


        // 2) Identify the closest cells to the desired start and end pose
        int start_x, start_y, goal_x, goal_y;
        costmap_->worldToMapEnforceBounds(start.pose.position.x, start.pose.position.y, start_x, start_y);
        costmap_->worldToMapEnforceBounds(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);

        ROS_WARN("Start_x: %d\tstart_y: %d", start_x, start_y);


        // 3) Use a wavefront algorithm to assign the distance transform value at each cell

        // Generate distance transform with BFS
        int height = costmap_->getSizeInCellsX();
        int width = costmap_->getSizeInCellsY();
        std::queue<cell> toVisit;
        bool visited[height][width] = { };
        int distanceTransform[height][width] = { };

        cell startCell = {start_x, start_y};
        toVisit.push(startCell);
        visited[start_x][start_y] = true;

        while (!toVisit.empty()) {
            cell c = toVisit.front();
            toVisit.pop();

            for (int i = std::max(c.x-1, 0); i < std::min(c.x+2, height); ++i) {
                for (int j = std::max(c.y-1, 0); j < std::min(c.y+2, width); ++j) {
                    if (!visited[i][j] && costmap_->getCost(i, j) < 25) {
                        cell newCell = {i, j};
                        toVisit.push(newCell);
                        distanceTransform[i][j] = distanceTransform[c.x][c.y] + 1;
                        visited[i][j] = true;
                    }
                }
            }
        }

        // 4) Iterate through these cells in a path of slowest decent as described in the paper
        bool cellsVisited[height][width] = { };
        cell c = {start_x, start_y};

        while (true) {

            // Find unvisited Neighbouring cell with highest DT
            cell maxCell = {c.x, c.y};
            int maxDT = 0;
            for (int i = std::max(c.x-1, 0); i < std::min(c.x+2, height); i++) {
                for (int j = std::max(c.y-1, 0); j < std::min(c.y+2, width); j++) {
                    if (i == c.x && j == c.y) continue;
                    if (distanceTransform[i][j] > maxDT && !cellsVisited[i][j]) {
                        maxCell.x = i;
                        maxCell.y = j;
                        maxDT = distanceTransform[i][j];
                    }
                }
            }

            // If No Neighbour Cell found then: Mark as Visited and Stop at Goal
            if (maxDT == 0) {
                cellsVisited[c.x][c.y] = true;
                ROS_WARN("No neighbour!");
                break;
            }

            // If Neighbouring Cell DT <= Current Cell DT then: Mark as Visited and Stop at Goal
            if (maxDT <= distanceTransform[c.x][c.y]) {
                cellsVisited[c.x][c.y] = true;
                if (c.x == start_x && c.y == start_y) {
                    break;
                }
            }

            // Set Current cell to Neigbouring cell
            c = maxCell;

            // Add to path
            geometry_msgs::PoseStamped newPoint = start;
            double wx, wy;
            costmap_->mapToWorld(c.x, c.y, wx, wy);
            newPoint.pose.position.x = wx;
            newPoint.pose.position.y = wy;

            plan.push_back(newPoint);

        }

        ROS_WARN("Size: %d", plan.size());

        ros::NodeHandle node("~/temp");
        ros::Publisher pub = node.advertise<nav_msgs::Path>("my_path", 1000);

        nav_msgs::Path thePath;
        thePath.header.stamp = start.header.stamp;
        thePath.header.frame_id = start.header.frame_id;
        thePath.poses = plan;

        pub.publish(thePath);

        return true;

    }

}
