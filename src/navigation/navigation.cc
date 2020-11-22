//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "simple_queue.h"
#include "shared/math/geometry.h"
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using geometry::line2f;

using namespace math_util;
using namespace ros_helpers;

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
VisualizationMsg local_viz_msg_;
VisualizationMsg global_viz_msg_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
      "ackermann_curvature_drive", 1);
  viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
  local_viz_msg_ = visualization::NewVisualizationMessage(
      "base_link", "navigation_local");
  global_viz_msg_ = visualization::NewVisualizationMessage(
      "map", "navigation_global");
  InitRosHeader("base_link", &drive_msg_.header);
  map_.Load(map_file);
  printf("Map Loaded! %d \n", int(map_.lines.size()));
  GenerateNavGraph();
  printf("Generate the map. (Edge num: %d) \n", AdjMap.edgeNum);
}

void Navigation::AddEdge2Graph(int vertex, int toVertex, float weight){
    EdgeNode *temp_edge  = new EdgeNode {toVertex, weight, NULL};
    if (AdjMap.adjList[vertex].firstEdge != NULL){
        temp_edge->next = AdjMap.adjList[vertex].firstEdge;
        AdjMap.adjList[vertex].firstEdge = temp_edge;
    }else {
        AdjMap.adjList[vertex].firstEdge = temp_edge;
    }
    AdjMap.edgeNum ++;
}

void Navigation::GenerateNavGraph() {
    // Initialize the AdjMap
    for (int i = 0; i < rows_ * cols_; ++i) {
        VertexNode temp_node {i, NULL};
        AdjMap.adjList.push_back(temp_node);
    }
    AdjMap.vertexNum = rows_ * cols_;

    std::pair<int, int> temp_cell_1;
    std::pair<int, int> temp_cell_2;
    for (int i = 0; i < rows_; ++i) {
        for (int j = 0; j < cols_; ++j) {
            temp_cell_1 = std::make_pair(i, j);
            // Grid
            if (i > 0){
                temp_cell_2 = std::make_pair(i - 1, j);
                if (!EdgeInterceptWall(temp_cell_1, temp_cell_2)){
                    AddEdge2Graph(Cell2Id(temp_cell_1), Cell2Id(temp_cell_2), 1);
                }
            }
            if (j > 0){
                temp_cell_2 = std::make_pair(i, j - 1);
                if (!EdgeInterceptWall(temp_cell_1, temp_cell_2)){
                    AddEdge2Graph(Cell2Id(temp_cell_1), Cell2Id(temp_cell_2), 1);
                }
            }
            if (i < rows_ - 1){
                temp_cell_2 = std::make_pair(i + 1, j);
                if (!EdgeInterceptWall(temp_cell_1, temp_cell_2)){
                    AddEdge2Graph(Cell2Id(temp_cell_1), Cell2Id(temp_cell_2), 1);
                }
            }
            if (j < cols_ - 1){
                temp_cell_2 = std::make_pair(i, j + 1);
                if (!EdgeInterceptWall(temp_cell_1, temp_cell_2)){
                    AddEdge2Graph(Cell2Id(temp_cell_1), Cell2Id(temp_cell_2), 1);
                }
            }
            // Uncomment these if want to have 8-connect grid
            if (i > 0 && j > 0){
                temp_cell_2 = std::make_pair(i - 1, j - 1);
                if (!EdgeInterceptWall(temp_cell_1, temp_cell_2)){
                    AddEdge2Graph(Cell2Id(temp_cell_1), Cell2Id(temp_cell_2), sqrt(2));
                }
            }
            if (i < rows_ - 1 && j > 0) {
                temp_cell_2 = std::make_pair(i + 1, j - 1);
                if (!EdgeInterceptWall(temp_cell_1, temp_cell_2)){
                    AddEdge2Graph(Cell2Id(temp_cell_1), Cell2Id(temp_cell_2), sqrt(2));
                }
            }
            if (i > 0 && j < cols_ - 1) {
                temp_cell_2 = std::make_pair(i - 1, j + 1);
                if (!EdgeInterceptWall(temp_cell_1, temp_cell_2)){
                    AddEdge2Graph(Cell2Id(temp_cell_1), Cell2Id(temp_cell_2), sqrt(2));
                }
            }
            if (i < rows_ - 1 && j < cols_ - 1){
                temp_cell_2 = std::make_pair(i + 1, j + 1);
                if (!EdgeInterceptWall(temp_cell_1, temp_cell_2)){
                    AddEdge2Graph(Cell2Id(temp_cell_1), Cell2Id(temp_cell_2), sqrt(2));
                }
            }
        }
    }
}

bool Navigation::EdgeInterceptWall(std::pair<int, int> cell_1, std::pair<int, int> cell_2) {
    Vector2f cell_coord_1 = Id2Coord(Cell2Id(cell_1)) + map_offset_;
    Vector2f cell_coord_2 = Id2Coord(Cell2Id(cell_2)) + map_offset_;
    line2f cell_line(cell_coord_1.x(), cell_coord_1.y(), cell_coord_2.x(), cell_coord_2.y());
    line2f map_line;
    for (unsigned long i = 0; i < map_.lines.size(); ++i) {
        map_line = map_.lines[i];
        if (map_line.Intersects(cell_line)){
            return true;
        }
    }
    return false;
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    // Print the global navigation goal coordinates
    Vector2f Map_loc = loc - map_offset_;
    printf("Set Navigation Goal at (%.2f, %.2f). \n", Map_loc.x(), Map_loc.y());
    printf("Navigation Cell is: (%d, %d).\n", Coord2Cell(loc).first, Coord2Cell(loc).second);
    printf("Navigation Cell Id is: %d \n", Cell2Id(Coord2Cell(loc)));
    printf("Navigation Cell loc is: (%.2f, %.2f). \n", Id2Coord(Cell2Id(Coord2Cell(loc))).x(), Id2Coord(Cell2Id(Coord2Cell(loc))).y());

    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    nav_complete_ = false;
    close2goal = false;
    // Clear the previous planned path and re-plan
    path_index.clear();
    MakePlan(robot_loc_, nav_goal_loc_);
    // Clear the previous global visualization info
    visualization::ClearVisualizationMsg(global_viz_msg_);
    // Draw the navigation goal
    visualization::DrawCross(loc, 0.2, 0x000000, global_viz_msg_);
    DrawPath();
}

void Navigation::DrawPath() {
    // Draw the Optimal Path
    printf("Path length: %d \n", int(path_index.size()));
    for (int i = 0; i < int(path_index.size()) - 1; ++i) {
        visualization::DrawLine(Id2Coord(path_index[i]) + map_offset_, Id2Coord(path_index[i+1]) + map_offset_, 0x0000FF, global_viz_msg_);
        visualization::DrawPoint(Id2Coord(path_index[i]) + map_offset_, 0x000000, global_viz_msg_);
    }
    printf("Path Start: %d,  End: %d \n", path_index[0], path_index[int(path_index.size()) - 1]);
}
void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    robot_loc_ = loc;
    robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
    odom_loc_ = loc;
    odom_angle_ = angle;
    robot_vel_ = vel;
    robot_omega_ = ang_vel;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    // Visualize the Cloud Point by Lidar Sensor
//    for(const auto & i : cloud){
//        visualization::DrawPoint({i.x(), i.y()}, 0x000000, local_viz_msg_);
//    }
    point_cloud_ = cloud;
//    // Vehicle's parameters
//    const double car_w = 0.281f; // Vehicle's width
//    const double car_l = 0.535f; // Vehicle's length
//    const double car_wheelbase = 0.324f; // Wheelbase
//    const double car_m = 0.15f; // Vehicle's Safety Margin
//    const double x_lim = 100.0f; // Max sensing length
//    const double pi_value = 3.1415926f; // PI
//    const double curvature_max = 1.0f; // Max Curvature
//    const double delta_curvature = 0.01f; // Curvature Increment
//    const double clearance_max = 100.f; // Max clearance
//    const double free_arc_angle_max = 2.0f * pi_value; // 2*PI
//    const double weight_free = 1.0f; // Weight factor for free arc length
//    const double weight_clear = 20.0f; // weight factor for clearance
//    const double weight_distance = -1.0f; // weight for distance to goal
//
//    // Initialize
//    vector<double> free_arc_length; // Free arc length
//    vector<double> min_clearance; // Minimum clearance
//    vector<double> distance2goal; // Distance to goal
//    double curvature = 0.0f;
//    double x = 0.0f;
//    double y = 0.0f;
//    double r = 0.0f;
//    double d = 0.0f;
//    double d_min = 0.0f;
//    double d_max = 0.0f;
//    double score = 0.0f;
//    double max_score = 0.0f;
//    unsigned int k = 0;
//    unsigned int max_index = 0;
//
//    // Compute free arc length
//    curvature = -curvature_max; // Curvature from -1 to 1
//
//    while(curvature <= curvature_max)
//    {
//        double min_arc_angle = free_arc_angle_max;
//        double min_arc_length = 0.0f;
//	    double clearance_min = clearance_max;
//
//        if (abs(curvature) > 0.001f)
//        {
//            r = abs(1.0f / curvature); // Radius
//            Vector2f circle_center = {0, 1/curvature}; // Circle center coordination
//            double distance = sqrt(3 * 3 + r * r) - r;
//            distance2goal.push_back(distance);
//            d_min = r - car_w * 0.5f - car_m;
//            d_max = sqrt((r + car_w * 0.5f + car_m) * (r + car_w * 0.5f + car_m) + ((car_l + car_wheelbase)/2 + car_m) * ((car_l + car_wheelbase)/2 + car_m));
//
//            unsigned int i = 0;
//
//            while (i < cloud.size())
//            {
//                x = cloud[i][0];
//                y = cloud[i][1];
//
//                if (abs(x) > 0.0f)
//                {
//                    d = sqrt(x * x + (circle_center[1] - y)*(circle_center[1] - y)); // Single Cloud Point to the Circle center
//
//                    if (d < d_min) // Calculate inner clearance
//                    {
//                        clearance_min = std::min(clearance_min, d_min - d);
//                    }
//		            else if (d > d_max) // Calculate outer clearance
//                    {
//                        clearance_min = std::min(clearance_min, d - d_max);
//                    }
//		            else // The point is on the way
//                    {
//                        if (x > 0.0f)
//                        {
//                            if (abs(y) < r)
//                            {
//                                min_arc_angle = std::min(asin(x / d), min_arc_angle);
//                            }
//                            else
//                            {
//                                min_arc_angle = std::min((pi_value - asin(x / d)), min_arc_angle);
//                            }
//                        }
//                        else
//                        {
//                            if (abs(y) > r)
//                            {
//                                min_arc_angle = std::min((pi_value + asin(abs(x) / d)), min_arc_angle);
//                            }
//                            else
//                            {
//                                min_arc_angle = std::min((2.0f * pi_value - asin(abs(x) / d)), min_arc_angle);
//                            }
//
//                        }
//                    }
//                }
//                i++;
//            }
//            min_arc_length = min_arc_angle * r;
//            float s_angle = 0.0f;
//            float e_angle = 0.0f;
//            if(curvature > 0){
//                s_angle = -pi_value/2;
//                e_angle = s_angle + min_arc_angle;
//            }else{
//                s_angle = pi_value/2 - min_arc_angle;
//                e_angle = pi_value/2;
//            }
//            visualization::DrawArc({0, 1/curvature}, r, s_angle, e_angle, 0xFF00FF, local_viz_msg_);
//        }
//        else
//        {
//            distance2goal.push_back(3.0f);
//            min_arc_length = x_lim;
//
//            unsigned int j = 0;
//
//            while (j < cloud.size())
//            {
//                x = cloud[j][0];
//                y = cloud[j][1];
//
//                if (x > 0.0f && abs(y) < (car_w * 0.5f + car_m))
//                {
//                    min_arc_length = std::min(x, min_arc_length);
//                }
//		        else if (x > 0.0f && y < - car_w * 0.5f - car_m)
//		        {
//                    clearance_min = std::min(clearance_min, - car_w * 0.5f - car_m - y);
//                }
//		        else if (x > 0.0f && y > car_w * 0.5f + car_m)
//		        {
//                    clearance_min = std::min(clearance_min, y - car_w * 0.5f - car_m);
//                }
//                j++;
//            }
//            visualization::DrawLine({0,0}, {min_arc_length, 0}, 0xFF00FF, local_viz_msg_);
//        }
//        if (min_arc_length > 100){
//            min_arc_length = 100;
//        }
//        free_arc_length.push_back(min_arc_length);
//	    min_clearance.push_back(clearance_min);
//
//        curvature = curvature + delta_curvature;
//    }
////    visualization::DrawCross(Vector2f(3, 0), 0.2, 0x000000, local_viz_msg_);
//    // Find best arc
//    while (k < free_arc_length.size())
//    {
//        score = weight_free * free_arc_length[k] + weight_clear * min_clearance[k] + weight_distance * distance2goal[k];
//
//        if (score > max_score)
//        {
//            max_score = score;
//            max_index = k;
//        }
//        k++;
//    }
//    double best_curvature = - curvature_max + delta_curvature * max_index;
//    if (abs(best_curvature) > 0.001){
//        double radius = abs(1/best_curvature);
//        visualization::DrawArc({0, 1/best_curvature}, radius, 0, 2 * pi_value, 0x0000, local_viz_msg_);
//    }else{
//        visualization::DrawLine({0,0}, {3,0}, 0x0000, local_viz_msg_);
//    }
//
//
////    viz_pub_.publish(local_viz_msg_);
//
//    // Execute motion on best arc
//    if (free_arc_length[max_index] > 0.01f)
//    {
//
//        drive_msg_.velocity = 1.0f;
////        drive_msg_.curvature = 0.1f;
//        drive_msg_.curvature =  - curvature_max + delta_curvature * max_index;
//
//        drive_msg_.velocity = 0.0f;
//        drive_msg_.curvature = 0.0f;
//    }
//    else
//    {
//        printf("Free arc length is too short!\n");
////        printf("");
//        drive_msg_.velocity = 0.0f;
//        drive_msg_.curvature = 0.0f;
//    }
//    drive_pub_.publish(drive_msg_);
}

std::pair<int, int> Navigation::Coord2Cell(Eigen::Vector2f coord){
    Vector2f actual_coord = coord - map_offset_;
    int row_index = actual_coord.y() / cell_size_;
    int col_index = actual_coord.x() / cell_size_;

    return std::make_pair(row_index, col_index);
}

int Navigation::Cell2Id(std::pair<int, int> cell){
    int CellId = cols_ * cell.first + cell.second;
    return CellId;
}

Vector2f Navigation::Id2Coord(int id){
    Vector2f coord;
    coord.y() = int(id / cols_) * cell_size_;
    coord.x() = id % rows_ * cell_size_;
    return coord;
}

void Navigation::MakePlan(Eigen::Vector2f start, Eigen::Vector2f end) {
    path_index.clear();

    std::pair<int, int> start_cell = Coord2Cell(start);
    std::pair<int, int> end_cell = Coord2Cell(end);
    printf("Start Cell: (%d, %d), End Cell: (%d, %d). \n", start_cell.first, start_cell.second, end_cell.first, end_cell.second);
    // Convert the coordinate to the cell id
    int start_cell_id = Cell2Id(start_cell);
    int end_cell_id = Cell2Id(end_cell);
    printf("Start Id: %d, End Id: %d \n", start_cell_id, end_cell_id);

    // Search for the optimal path
    Dijkstra(start_cell_id, end_cell_id);
    //
    printf("Make plan successfully! \n");
}

void Navigation::Dijkstra(int startVertexId, int goalVertexId) {
    int VERTEX_NUM = AdjMap.vertexNum;
    float MAX_COST = 1000000;
    SimpleQueue<int, float> frontier;
    frontier.Push(startVertexId, 0);
    std::map<int, int> parent;
    parent[startVertexId] = -1;
    std::map<int, float> cost;
    for (int i = 0; i < VERTEX_NUM; ++i) {
        cost[i] = MAX_COST;
    }
    cost[startVertexId] = 0;

    while (!frontier.Empty()){
        int current = frontier.Pop();
        if (current == goalVertexId){
            printf("Path Found!\n");
            break;
        }
        auto Edge2nextVertex = AdjMap.adjList[current].firstEdge;
        while (Edge2nextVertex != NULL){
            float new_cost = cost[current] + Edge2nextVertex->weight;
            if (new_cost < cost[Edge2nextVertex->toAdjVex]){
                cost[Edge2nextVertex->toAdjVex] = new_cost;
                frontier.Push(Edge2nextVertex->toAdjVex, new_cost);
                parent[Edge2nextVertex->toAdjVex] = current;
            }
            Edge2nextVertex = Edge2nextVertex->next;
        }
    }
    int current = goalVertexId;
    while (current != startVertexId){
        path_index.push_back(current);
        current = parent[current];
    }
    path_index.push_back(startVertexId);
    std::reverse(path_index.begin(), path_index.end());
}

bool Navigation::PathStillValid() {
    std::reverse(path_index.begin(), path_index.end());
    for (int i = 0; i < int(path_index.size()); ++i) {
        // Check whether the robot is close to the planned path
        if ((Id2Coord(path_index[i]) + map_offset_ - robot_loc_).norm() < valid_tolerance_){
            std::reverse(path_index.begin(), path_index.end());
            return true;
        }
    }
    std::reverse(path_index.begin(), path_index.end());
    return false;
//    printf("First Path: %d, robot loc (%.2f, %.2f) \n", path_index[0], robot_loc_.x(), robot_loc_.y());
//    printf("start loc (%.2f, %.2f) \n", (Id2Coord(path_index[0]) + map_offset_).x(), (Id2Coord(path_index[0]) + map_offset_).y());
//    printf("Valid %.2f \n", (Id2Coord(path_index[0]) + map_offset_ - robot_loc_).norm());
//    return true;
}

bool Navigation::ReachedGoal() {
    float dis2goal = (nav_goal_loc_ - robot_loc_).norm();
    if (dis2goal < 0.8){
        close2goal = true;
    }
    if (dis2goal <= reach_tolerance_){
        nav_complete_ = true;
        drive_msg_.velocity = 0.0f;
        drive_msg_.curvature = 0.0f;
        drive_pub_.publish(drive_msg_);
        printf("We are reaching the goal! \n");
        return true;
    } else {
        return false;
    }
}

void Navigation::GetCarrot() {
//    printf("Update Carrot! \n");
    std::reverse(path_index.begin(), path_index.end());
    // Check whether the robot is close to the goal
    if ((Id2Coord(path_index[0]) + map_offset_ - robot_loc_).norm() < carrot_radius) {
        carrot_ = Id2Coord(path_index[0]);
        visualization::DrawCross(carrot_ + map_offset_, 0.1, 0xFF0000, global_viz_msg_);
        std::reverse(path_index.begin(), path_index.end());
        return;
    }
    for (int i = 1; i < int(path_index.size()); ++i) {
        if ((Id2Coord(path_index[i]) + map_offset_ - robot_loc_).norm() < carrot_radius){
            carrot_ = Id2Coord(path_index[i - 1]);
            visualization::DrawCross(carrot_ + map_offset_, 0.1, 0xFF0000, global_viz_msg_);
            std::reverse(path_index.begin(), path_index.end());
            return;
        }
    }
}

Eigen::Vector2f Navigation::Global2Local(Eigen::Vector2f global_loc){
    // Transfer the global location to the map location
    Vector2f map_loc = global_loc + map_offset_;

    Eigen::Rotation2Df Rotate2Local(-robot_angle_);
    Vector2f local_loc = Rotate2Local * (map_loc - robot_loc_);
    return local_loc;
}

void Navigation::RunObstacleAvoidance() {
    path_option_.clear();
    // Set carrot as the local navigation target
    Vector2f local_carrot_loc = Global2Local(carrot_);
    visualization::DrawCross(local_carrot_loc, 0.1, 0xFFFF00, local_viz_msg_);
    // Vehicle's parameters
    const double car_w = 0.281f; // Vehicle's width
    const double car_l = 0.535f; // Vehicle's length
    const double car_wheelbase = 0.324f; // Wheelbase
    const double car_m = 0.1f; // Vehicle's Safety Margin
    const double x_lim = 100.0f; // Max sensing length
    const double pi_value = 3.1415926f; // PI
    const double curvature_max = 1.0f; // Max Curvature
    const double delta_curvature = 0.02f; // Curvature Increment
    const double clearance_max = 1.0f; // Max clearance
    const double free_arc_angle_max = 0.6 * pi_value; // 2*PI
    const double weight_free = 8.0f; // Weight factor for free arc length
    const double weight_clear = 14.0f; // weight factor for clearance
    const double weight_distance = -23.0f; // weight for distance to goal

    // Initialize
    double curvature;
    double x;
    double y;
    double r;
    double d;
    double d_min;
    double d_max;
    double score;
    double max_score = -10000.0f;
    unsigned int max_index = 0;

    // Compute free arc length
    curvature = -curvature_max; // Curvature from -1 to 1

    while(curvature <= curvature_max) {
        PathOption temp_path_option;
        temp_path_option.curvature = curvature;

        double min_arc_angle = free_arc_angle_max;
        double min_arc_length;
        double clearance_min = clearance_max;
        // Curve Path
        if (abs(curvature) > 0.001f) {
            // In Robot Local coordinate
            r = abs(1.0f / curvature); // Radius
            Vector2f circle_center = {0, 1/curvature}; // Circle center coordination
            // Distance to the carrot
            double distance = abs((circle_center - local_carrot_loc).norm() - r);
            temp_path_option.distance2goal = distance;

            d_min = r - car_w * 0.5f - car_m;
            d_max = sqrt((r + car_w * 0.5f + car_m) * (r + car_w * 0.5f + car_m) + ((car_l + car_wheelbase)/2 + car_m) * ((car_l + car_wheelbase)/2 + car_m));

            for (unsigned int i = 0; i < point_cloud_.size(); ++i){
                if (point_cloud_[i].norm() > 5){
                    continue;
                }
                x = point_cloud_[i].x();
                y = point_cloud_[i].y();

                if (abs(x) > 0.0f) {
                    d = sqrt(x * x + (circle_center.y() - y)*(circle_center.y() - y)); // Single Cloud Point to the Circle center
                    // Calculate inner clearance
                    if (d < d_min) {
                        clearance_min = std::min(clearance_min, d_min - d);
                    } else if (d > d_max) {
                        // Calculate outer clearance
                        clearance_min = std::min(clearance_min, d - d_max);
                    } else {
                        // The point is on the way
                        if (x > 0.0f) {
                            if (abs(y) < r) {
                                min_arc_angle = std::min(asin(x / d), min_arc_angle);
                            } else {
                                min_arc_angle = std::min((pi_value - asin(x / d)), min_arc_angle);
                            }
                        } else {
                            if (abs(y) > r) {
                                min_arc_angle = std::min((pi_value + asin(abs(x) / d)), min_arc_angle);
                            } else {
                                min_arc_angle = std::min((2.0f * pi_value - asin(abs(x) / d)), min_arc_angle);
                            }
                        }
                        min_arc_angle = std::min(min_arc_angle, pi_value);
                    }
                }
            }

            min_arc_length = min_arc_angle * r;
//            float s_angle;
//            float e_angle;
//            if(curvature > 0){
//                s_angle = -pi_value/2;
//                e_angle = s_angle + min_arc_angle;
//            }else{
//                s_angle = pi_value/2 - min_arc_angle;
//                e_angle = pi_value/2;
//            }
//            visualization::DrawArc({0, 1/curvature}, r, s_angle, e_angle, 0xFF00FF, local_viz_msg_);
        // Straight Path
        } else {
            temp_path_option.distance2goal = abs(local_carrot_loc.y());
            min_arc_length = x_lim;

            for (unsigned int j = 0; j < point_cloud_.size(); ++j) {
                x = point_cloud_[j].x();
                y = point_cloud_[j].y();

                if (x > 0.0f && abs(y) < (car_w * 0.5f + car_m)) {
                    min_arc_length = std::min(x, min_arc_length);
                } else if (x > 0.0f && y < - car_w * 0.5f - car_m) {
                    clearance_min = std::min(clearance_min, - car_w * 0.5f - car_m - y);
                } else if (x > 0.0f && y > car_w * 0.5f + car_m) {
                    clearance_min = std::min(clearance_min, y - car_w * 0.5f - car_m);
                }
            }

//            visualization::DrawLine({0,0}, {min_arc_length, 0}, 0xFF00FF, local_viz_msg_);
        }
        if (min_arc_length > 3){
            min_arc_length = 3;
        }
        temp_path_option.free_path_length = min_arc_length;
        temp_path_option.clearance = clearance_min;
        path_option_.push_back(temp_path_option);

        curvature = curvature + delta_curvature;
    }
    // Find best arc
    for (unsigned int k = 0; k < path_option_.size(); ++k) {
        // If robot is very close to the goal, directly choose the closest path
        if (close2goal){
            score = weight_distance * path_option_[k].distance2goal;
            path_option_[k].score = score;
        } else {
            if (path_option_[k].free_path_length <= 0.5 || path_option_[k].clearance <= 0.01){
                continue;
            }
            score = weight_free * path_option_[k].free_path_length + weight_clear * path_option_[k].clearance + weight_distance * path_option_[k].distance2goal;
            path_option_[k].score = score;
        }
        if (score > max_score) {
            max_score = score;
            max_index = k;
        }
//        printf("Path Option %d: (%.2f, %.2f, %2f, %.2f) Score: %.2f \n", k, path_option_[k].curvature,
//               path_option_[k].clearance, path_option_[k].distance2goal, path_option_[k].free_path_length, path_option_[k].score);
    }

    printf("Best Path Option: (%.2f, %.2f, %2f, %.2f) \n", path_option_[max_index].curvature, path_option_[max_index].clearance, path_option_[max_index].distance2goal, path_option_[max_index].free_path_length);
    visualization::DrawPathOption(path_option_[max_index].curvature, path_option_[max_index].free_path_length, path_option_[max_index].clearance, local_viz_msg_);
    double best_curvature = - curvature_max + delta_curvature * max_index;
//    if (abs(best_curvature) > 0.001){
//        double radius = abs(1/best_curvature);
//        visualization::DrawArc({0, 1/best_curvature}, radius, 0, 2 * pi_value, 0x000000, local_viz_msg_);
//    } else {
//        visualization::DrawLine({0,0}, {3,0}, 0x000000, local_viz_msg_);
//    }

    // Execute motion on best arc
    if (path_option_[max_index].free_path_length > 1.0f) {
        drive_msg_.velocity = 0.5f;
        drive_msg_.curvature =  best_curvature;
    } else if (path_option_[max_index].free_path_length > 0.01f) {
        drive_msg_.velocity = std::max(0.4f * path_option_[max_index].free_path_length, 0.1f);
        drive_msg_.curvature =  std::min(best_curvature * path_option_[max_index].free_path_length, best_curvature * 0.5);
        printf("SLow Down. \n");
    } else {
        printf("Free arc length is too short!\n");
        drive_msg_.velocity = 0.0f;
        drive_msg_.curvature = 0.0f;
    }
    drive_pub_.publish(drive_msg_);

}

void Navigation::Run() {
    static vector<float> velocity_log;
    static vector<float> curvature_log;

    visualization::ClearVisualizationMsg(local_viz_msg_);
    // Check whether the robot reach the target point
    if (nav_complete_ || ReachedGoal()) {
        return;
    }
    // Check whether the planned path is still valid
    if (!PathStillValid()) {
        printf("The planned path is invalid! \n");
        // Re-plan the path
        visualization::ClearVisualizationMsg(global_viz_msg_);
        // Draw the navigation goal
        visualization::DrawCross(nav_goal_loc_, 0.2, 0x000000, global_viz_msg_);
        MakePlan(robot_loc_, nav_goal_loc_);
        DrawPath();
    }
    // Get and Draw the carrot
    GetCarrot();
    // Test whether Global2Local function is correct
//    Vector2f temp_loc = Global2Local(carrot_);
//    visualization::DrawCross(temp_loc, 0.1, 0xFFFF00, local_viz_msg_);
//    viz_pub_.publish(local_viz_msg_);
    // Run Obstacle Avoidance
    RunObstacleAvoidance();
//    for(const auto & i : point_cloud_){
//        visualization::DrawPoint({i.x(), i.y()}, 0x000000, local_viz_msg_);
//    }
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);

  // Create Helper functions here
//  visualization::ClearVisualizationMsg(local_viz_msg_);

////  drive_msg_.velocity = 1;
////  drive_msg_.curvature = 0;
////  drive_pub_.publish(drive_msg_);
//    visualization::DrawCross(Vector2f(3, 0), 0.2, 0xFF0000, local_viz_msg_);
//    visualization::DrawArc({0,0}, 10, 30, 90, 0xFF0000, local_viz_msg_);
//    viz_pub_.publish(local_viz_msg_);
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
}

}  // namespace navigation
