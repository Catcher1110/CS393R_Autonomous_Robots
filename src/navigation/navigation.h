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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

typedef struct EdgeNode {
    int toAdjVex; // The index of vertex array which this edge points to
    float weight; // The edge weight
    struct EdgeNode *next; // The next edge, note that it only means the next edge also links to the vertex which this
                           // edge links to
} EdgeNode;

typedef struct VertexNode {
    int VertexId; // Vertex Id
    struct EdgeNode* firstEdge; // The first edge which the vertex points to
} VertexNode;

typedef struct {
    std::vector<VertexNode> adjList; // Adjacency list, which stores the all vertex
    int vertexNum; // The number of vertex
    int edgeNum; // The number of edge
} AdjListGraph;

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  float distance2goal;
  float score;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continuously from main
  void Run();
  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);

  // Run Obstacle Avoidance
  void RunObstacleAvoidance();
  // Change the global carrot coordinate to the robot coordinate
  Eigen::Vector2f Global2Local(Eigen::Vector2f global_loc);

  // Create the navigation graph
  void GenerateNavGraph();
  // Check whether the edge intercept with the wall
  bool EdgeInterceptWall(std::pair<int, int> cell_1, std::pair<int, int> cell_2);
  // Add the edge to the graph
  void AddEdge2Graph(int vertex, int toVertex, float weight);

  // Check Whether Reached Goal
  bool ReachedGoal();

  // Execute Path Planning
  void MakePlan(Eigen::Vector2f start, Eigen::Vector2f end);

  // Dijkstra Algorithm
  void Dijkstra(int startVertexId, int goalVertexId);

  // Draw Optimal Path
  void DrawPath();
  // Get the carrot location
  void GetCarrot();

  // Check whether the path is valid
  bool PathStillValid();

  // Convert the coordinate to the cell
  std::pair<int, int> Coord2Cell(Eigen::Vector2f coord);
  // Convert the cell to the id
  int Cell2Id(std::pair<int, int> cell);
  // Convert the id to the coord
  Eigen::Vector2f Id2Coord(int id);

 private:

  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;

  //
  std::vector<PathOption> path_option_;
  std::vector<Eigen::Vector2f> point_cloud_;
  vector_map::VectorMap map_;
  // Use Adjacency list to store the map graph
  AdjListGraph AdjMap;
  // Whether navigation is complete.
  bool nav_complete_;
  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;

  // Reach Goal tolerance
  bool close2goal;
  double reach_tolerance_ = 0.6;
  //
  std::vector<int> path_index;
  Eigen::Vector2f carrot_;
  double carrot_radius = 1.5;
  // Grid
  Eigen::Vector2f map_offset_ {-50, -50};
  double const cell_size_ = 0.25;
  int cols_ = 400;
  int rows_ = 400;
  // Plan Valid Checking tolerance
  double valid_tolerance_ = 0.5;
};

}  // namespace navigation

#endif  // NAVIGATION_H
