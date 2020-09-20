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
#include "visualization/visualization.h"

using Eigen::Vector2f;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;

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
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
    const double car_w = 0.281f; // Vehicle's width
    const double car_l = 0.535f; // Vehicle's length
    const double car_m = 0.01f; // Vehicle's Safety Margin
    const double x_lim = 100.0f; // Max sensing length
    const double pi_value = 3.1415926f;
    const double curvature_max = 1.0f; // Max Curvature
    const double delta_curvature = 0.02f; // Curvature Increment
    const double free_arc_angle_max = 2.0f * pi_value;
    const double weight_free = 1.0f;
    const double weight_clearance = 1.0f;

    // Initialize
    vector<double> free_arc_length;
    vector<double> inner_clearance; // Clearance for inner side
    vector<double> outer_clearance; // Clearance for outer side
    vector<double> clearance; // Clearance of min(inner, outer)
    vector<double> distance2goal; // Distance to goal
    double curvature = 0.0f;
    double x = 0.0f;
    double y = 0.0f;
    double r = 0.0f;
    double d = 0.0f;
    double d_min = 0.0f;
    double d_max = 0.0f;
    double score = 0.0f;
    double max_score = 0.0f;
    unsigned int k = 0;
    unsigned int max_index = 0;

    // Compute free arc length
    curvature = -curvature_max; // Curvature from -1 to 1

    while(curvature <= curvature_max){
        double min_arc_angle = free_arc_angle_max;
        double min_arc_length = 0.0f;

        double min_inner_clearance = 100;
        double min_outer_clearance = 100;

        if (abs(curvature) > 0.001f)
        {
            r = abs(1.0f / curvature);
            d_min = r - car_w * 0.5f - car_m;
            d_max = sqrt((r + car_w * 0.5f + car_m) * (r + car_w * 0.5f + car_m) + (car_l + car_m) * (car_l + car_m));

            unsigned int i = 0;

            while (i < cloud.size())
            {
                x = cloud[i][0];
                y = cloud[i][1];

                if (abs(x) > 0.0f)
                {
                    d = sqrt(x * x + (r - y)*(r - y));

                    if (d < d_min) // Calculate inner clearance
                    {
                        min_inner_clearance = std::min(min_inner_clearance, d_min - d);
                    }else if (d > d_max) // Calculate outer clearance
                    {
                        min_outer_clearance = std::min(min_outer_clearance, d - d_max);
                    }else // The point is on the way
//                    if ((d > d_min) && (d < d_max))
                    {
                        if (x > 0.0f)
                        {
                            if (y < r)
                            {
                                min_arc_angle = std::min(asin(x / d), min_arc_angle);
                            }
                            else
                            {
                                min_arc_angle = std::min((pi_value - asin(x / d)), min_arc_angle);
                            }
                        }
                        else
                        {
                            if (y > r)
                            {
                                min_arc_angle = std::min((pi_value + asin(abs(x) / d)), min_arc_angle);
                            }
                            else
                            {
                                min_arc_angle = std::min((2.0f * pi_value - asin(abs(x) / d)), min_arc_angle);
                            }

                        }
                    }
                }
                i++;
            }
            min_arc_length = min_arc_angle * r;
        }
        else
        {
            min_arc_length = x_lim;

            unsigned int j = 0;

            while (j < cloud.size())
            {
                x = cloud[j][0];
                y = cloud[j][1];

                if (x > 0.0f && abs(y) < (car_w * 0.5f + car_m))
                {
                    min_arc_length = std::min(x, min_arc_length);
                }else if (x > 0.0f && y < - car_w * 0.5f - car_m){
                    min_inner_clearance = std::min(min_inner_clearance, - car_w * 0.5f - car_m - y);
                }else if (x > 0.0f && y > car_w * 0.5f + car_m){
                    min_outer_clearance = std::min(min_outer_clearance, y - car_w * 0.5f - car_m);
                }
                j++;
            }

        }

        free_arc_length.push_back(min_arc_length);
        inner_clearance.push_back(min_inner_clearance);
        outer_clearance.push_back(min_outer_clearance);
        clearance.push_back(std::min(min_inner_clearance, min_outer_clearance));
        curvature = curvature + delta_curvature;
    }

    // Find best arc
    while (k < free_arc_length.size())
    {
        score = weight_free * free_arc_length[k] + weight_clearance * clearance[k];

        if (score > max_score)
        {
            max_score = score;
            max_index = k;
        }
        k++;
    }

    // Execute motion on best arc
    if (free_arc_length[max_index] > 0.06f)
    {

        drive_msg_.velocity = 1.0f;
        drive_msg_.curvature =  - curvature_max + delta_curvature * max_index;
    }
    else
    {
        drive_msg_.velocity = 0.0f;
        drive_msg_.curvature = 0.0f;
    }

    drive_pub_.publish(drive_msg_);
}

void Navigation::Run() {
  // Create Helper functions here
  visualization::ClearVisualizationMsg(local_viz_msg_);
//  drive_msg_.header = amrl_msgs::AckermannCurvatureDriveMsg_<>;
//  drive_msg_.velocity = 1;
//  drive_msg_.curvature = 0;
//  drive_pub_.publish(drive_msg_);
    visualization::DrawCross(Vector2f(3, 0), 0.2, 0xFF0000, local_viz_msg_);
    visualization::DrawArc({10,10}, 10, 30, 90, 0xFF0000, local_viz_msg_);
    viz_pub_.publish(local_viz_msg_);
  // Milestone 1 will fill out part of this class.
  // Milestone 3 will complete the rest of navigation.
}

}  // namespace navigation
