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

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;

namespace slam {

    SLAM::SLAM() :
            prev_odom_loc_(0, 0),
            prev_odom_angle_(0),
            odom_initialized_(false),
            car_loc(0, 0), // updated car's location through estimation
            car_angle(0),
            odom_car_loc(0, 0), // updated car's location through odometry
            odom_car_angle(0),
            map_car_loc(0, 0),
            map_car_angle(0) {}

    void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) {
        // Return the latest pose estimate of the robot.

        // Definition and initialization
        const Vector2f loc_odom = odom_car_loc;
        const double ang_odom = odom_car_angle;
        const double delta_x = 0.5f;
        const double max_delta_x = delta_x * 5.0f; // 0.5f
        const double delta_y = 0.5f;
        const double max_delta_y = delta_y * 5.0f;
        const double delta_ang = 3.1415926 / 40.0;
        const double max_delta_ang = delta_ang * 5.0f;

        double loc_x_optimal = loc_odom.x();
        double loc_y_optimal = loc_odom.y();
        double angle_optimal = ang_odom;

        double loc_x = loc_odom.x() - max_delta_x;
        double max_prob = 0.0f;
//        if (point_cloud_.empty()) {
//            printf("GetPose point_cloud_ empty.\n");
//            *loc = Vector2f(loc_x_optimal, loc_y_optimal);
//            *angle = angle_optimal;
//            return;
//        } else{
//            printf("GetPose point_cloud_ %d. \n", int(point_cloud_.size()));
//        }
        Vector2f translation_back;
        double rotation_back;
        vector<Vector2f> point_cloud_rotated_back;

        if (!costTable2D.empty())
        {
//            printf("cost table size: %d. \n", int(costTable2D.size()));
            // Sample all possible location & angle near odometry predicted location & angle
            while (loc_x < (loc_odom.x() + max_delta_x)) {

                double loc_y = loc_odom.y() - max_delta_y;

                while (loc_y < (loc_odom.y() + max_delta_y)) {

                    double angle_t = ang_odom - max_delta_ang;

                    while (angle_t < (ang_odom + max_delta_ang)) {

                        double prob = 0.0f;

                        translation_back.x() = car_loc.x() - loc_x;
                        translation_back.y() = car_loc.y() - loc_y;
                        rotation_back = car_angle - angle_t;

                        // Transform current point cloud back to ref (last step) frame
                        point_cloud_rotated_back = PointCloudRotation(point_cloud_, translation_back, rotation_back);

                        // Compute cost (probability)
                        prob = computeCost(point_cloud_rotated_back);
//                        printf("prob for %.2f and %.2f is %.2f. \n", loc_x, loc_y, prob);
                        if (prob > max_prob) {
                            max_prob = prob;
                            loc_x_optimal = loc_x;
                            loc_y_optimal = loc_y;
                            angle_optimal = angle_t;
                        }
                        angle_t = angle_t + delta_ang;
                    }
                    loc_y = loc_y + delta_y;
                }
                loc_x = loc_x + delta_x;
            }
        }
//        printf("After While loop \n");
        printf("Current Angle: %.2f, Pose: %.2f, %.2f. \n", angle_optimal, loc_x_optimal, loc_y_optimal);
        *loc = Vector2f(loc_x_optimal, loc_y_optimal);
        *angle = angle_optimal;

        car_loc.x() = loc_x_optimal;
        car_loc.y() = loc_y_optimal;
        car_angle = angle_optimal;

        // Update Cost table
        costTable2D = generateCostTable2D(point_cloud_);
    }

    void SLAM::ObserveLaser(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max) {
        // A new laser scan has been observed. Decide whether to add it as a pose
        // for SLAM. If decided to add, align it to the scan from the last saved pose,
        // and save both the scan and the optimized pose.
        printf("Observe Laser. \n");
        // Observe lidar and transform point cloud into vehicle frame
        point_cloud_.clear();
        Vector2f single_point_cloud = {0.0f, 0.0f};
        unsigned int i = 0;

        double angle_increment = (angle_max - angle_min) / ranges.size();

        while (i < (ranges.size()))
        {
            if ((ranges[i] < range_max) && (ranges[i] > range_min))
            {
                single_point_cloud = {ranges[i] * cos(angle_min + angle_increment * i) + 0.2f, ranges[i] * sin(angle_min + angle_increment * i)};
                point_cloud_.push_back(single_point_cloud);
            }
            i++;
        }

    }

    void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
//        printf("Odometry loc: %.2f, %.2f. \n", odom_loc.x(), odom_loc.y());
        if (!odom_initialized_)
        {
            prev_odom_angle_ = odom_angle;
            prev_odom_loc_ = odom_loc;
            odom_initialized_ = true;
            return;
        }
            // Keep track of odometry to estimate how far the robot has moved between
            // poses.
        else
        {
            Eigen::Rotation2Df rotate_odom_prev(- prev_odom_angle_);
            Vector2f delta_T = rotate_odom_prev * (odom_loc - prev_odom_loc_);

            // Update car's location in ground frame according to odom measurement (motion model)
            Eigen::Rotation2Df rotate_map_prev(car_angle);
            odom_car_loc = car_loc + rotate_map_prev * delta_T;
            odom_car_angle = car_angle + odom_angle - prev_odom_angle_;
            printf("Odometry loc: %.2f, %.2f; Car loc: %.2f, %.2f. \n", odom_car_loc.x(), odom_car_loc.y(), car_loc.x(), car_loc.y());
            prev_odom_loc_ = odom_loc;
            prev_odom_angle_ = odom_angle;
        }
    }

    vector<Vector2f> SLAM::GetMap() {
        vector<Vector2f> map;
        // Reconstruct the map as a single aligned point cloud from all saved poses
        // and their respective scans.
        if (point_cloud_.empty()){
            printf("GetMap point_cloud_ empty. \n");
            return map_global;
        }
        if (odom_initialized_){
            if ((map_car_loc - car_loc).norm() < 0.5 && abs(map_car_angle - car_angle) < 30){
                printf("Do not update map.\n");
                return map_global;
            }
        }
        vector<Vector2f> point_cloud_rotated_back_origin;

        // Rotate current point cloud back to original frame (0, 0, 0)
        point_cloud_rotated_back_origin = PointCloudRotation(point_cloud_, -car_loc, -car_angle);

        unsigned int i = 0;

        while (i < point_cloud_rotated_back_origin.size())
        {
            map_global.push_back(point_cloud_rotated_back_origin[i]);
            i++;
        }

        map = map_global;

        map_car_loc = car_loc;
        map_car_angle = car_angle;
        return map;
    }

// Create 2D Cost Table (Previous car's frame)

    std::vector<std::vector<double>> SLAM::generateCostTable2D(const std::vector<Eigen::Vector2f>& point_cloud) {
        printf("Generate cost table.\n");
        std::vector<std::vector<double>> cost_table;
        cost_table.clear();

        const double x_max = 5.0f;
        const double y_max = 5.0f;
        const double delta_x = 0.05f;
        const double delta_y = 0.05f;
        const double sigma = 1.0f;

        double x = - x_max;
        double y;
        double epsilon = 0.1f;
        std::vector<double> cost_table_row;
        while (x <= x_max + epsilon)
        {
            y = - y_max;
            while (y <= y_max + epsilon)
            {
                double cost = 0.0f;
                unsigned int i = 0;

                while (i < point_cloud.size())
                {
                    double distance = (Vector2f(x, y) - point_cloud[i]).norm();
                    cost += exp(- distance * distance / sigma);
                    i++;
                }

                cost_table_row.push_back(cost);
                y = y + delta_y;
//                printf("y = %.2f .\n", y);
            }
            cost_table.push_back(cost_table_row);
            cost_table_row.clear();
            x = x + delta_x;
//            printf("x = %.2f .\n", x);
        }

        return cost_table;
    }

// 2D Look-up Cost (transform current car's point cloud to reference car's frame, then compute cost based on the difference between current point cloud and reference cloud)

    double SLAM::computeCost(const std::vector<Eigen::Vector2f>& point_cloud) {

        const double x_max = 5.0f;
        const double y_max = 5.0f;
        const double delta_x = 0.05f;
        const double delta_y = 0.05f;

        double cost = 0.0f;

        unsigned int i = 0;

        while (i < point_cloud.size())
        {
            if ((std::abs(point_cloud[i].x()) >  x_max) || std::abs(point_cloud[i].y()) >  y_max)
            {
                cost = cost + 0.0f;
            }
            else
            {
                unsigned int j = 0;
                unsigned int k = 0;
                unsigned int one = 1;
                double x_left = - x_max;
                double y_left = - y_max;

                // Find index in 2D table for bi-linear interpolation
                while (j < costTable2D.size() - one)
                {
                    if ((point_cloud[i].x() >  x_left) && (point_cloud[i].x() <  x_left + delta_x))
                    {
                        break;
                    }
                    else
                    {
                        x_left = x_left + delta_x;
                        j++;
                    }
                }
//                printf("After j while loop. j = %d.\n", j);

                while (k < costTable2D[0].size() - one)
                {
                    if ((point_cloud[i].y() >  y_left) && (point_cloud[i].y() <  y_left + delta_y))
                    {
                        break;
                    }
                    else
                    {
                        y_left = y_left + delta_y;
                        k++;
                    }
                }
//                printf("After k while loop. k = %d. \n", k);
//                printf("cost table size. %d.\n", int(costTable2D[1].size()));

                j = std::min(j, (unsigned int) (costTable2D.size() - one * 2));
                k = std::min(k, (unsigned int) (costTable2D[0].size() - one * 2));
//                printf("j = %d. \n", j);
//                printf("k = %d. \n", k);
                double x_right = x_left + delta_x;
                double y_right = y_left + delta_y;

                // Bi-linear interpolation
                double Q_11 = costTable2D[j][k];
                double Q_21 = costTable2D[j + one][k];
                double Q_12 = costTable2D[j][k + one];
                double Q_22 = costTable2D[j + one][k + one];

                double L_1 = ((point_cloud[i].x() - x_left) / delta_x) * Q_11 + ((x_right - point_cloud[i].x()) / delta_x) * Q_21;
                double L_2 = ((point_cloud[i].x() - x_left) / delta_x) * Q_12 + ((x_right - point_cloud[i].x()) / delta_x) * Q_22;

                cost = cost + ((point_cloud[i].y() - y_left) / delta_y) * L_1 + ((y_right - point_cloud[i].y()) / delta_y) * L_2;
            }

            i++;
        }

        return cost;
    }

    // Point cloud rotation (From current frame to reference (previous time step) frame)

    std::vector<Eigen::Vector2f> SLAM::PointCloudRotation(const std::vector<Eigen::Vector2f>& point_cloud, const Eigen::Vector2f& translation, const double rotation)
    {
        vector<Vector2f> rotated_point_cloud;

        unsigned int i = 0;

        while (i < point_cloud.size())
        {
            Vector2f rotated_point = point_cloud[i] + translation;
            Vector2f rotated_point_temp = rotated_point;

            rotated_point.x() = cos(rotation) * rotated_point_temp.x() - sin(rotation) * rotated_point_temp.y();
            rotated_point.y() = sin(rotation) * rotated_point_temp.x() + cos(rotation) * rotated_point_temp.y();

            rotated_point_cloud.push_back(rotated_point);
            i++;
        }

        return rotated_point_cloud;
    }

}  // namespace slam
