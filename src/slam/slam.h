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
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

    class SLAM {

    public:

        // Default Constructor.
        SLAM();

        // Observe a new laser scan.
        void ObserveLaser(const std::vector<float>& ranges,
                          float range_min,
                          float range_max,
                          float angle_min,
                          float angle_max);

        // Observe new odometry-reported location.
        void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                             const float odom_angle);

        // Get latest map.
        std::vector<Eigen::Vector2f> GetMap();

        // Get latest robot pose.
        void GetPose(Eigen::Vector2f* loc, float* angle);


    private:

        std::vector<std::vector<double>> generateCostTable2D(const std::vector<Eigen::Vector2f>& point_cloud);

        double computeCost(const std::vector<Eigen::Vector2f>& point_cloud);

        std::vector<Eigen::Vector2f> PointCloudRotation(const std::vector<Eigen::Vector2f>& point_cloud, const Eigen::Vector2f& translation, const double rotation);

        // Previous odometry-reported locations.
        Eigen::Vector2f prev_odom_loc_;
        float prev_odom_angle_;
        bool odom_initialized_;

        // Internal Variables
        std::vector<Eigen::Vector2f> point_cloud_;
        Eigen::Vector2f car_loc;
        double car_angle;
        Eigen::Vector2f odom_car_loc;
        double odom_car_angle;
        std::vector<std::vector<double>> costTable2D;

        // Map
        std::vector<Eigen::Vector2f> map_global;
        Eigen::Vector2f map_car_loc;
        double map_car_angle;
    };
}  // namespace slam

#endif   // SRC_SLAM_H_