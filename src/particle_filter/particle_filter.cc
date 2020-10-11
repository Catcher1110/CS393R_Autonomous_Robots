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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

    config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

    ParticleFilter::ParticleFilter() :
            prev_odom_loc_(0, 0),
            prev_odom_angle_(0),
            odom_initialized_(false) {}

    void ParticleFilter::GetParticles(vector<Particle>* particles) const {
        *particles = particles_;
    }

    void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                                const float angle,
                                                int num_ranges,
                                                float range_min,
                                                float range_max,
                                                float angle_min,
                                                float angle_max,
                                                vector<Vector2f>* scan_ptr) {
        vector<Vector2f>& scan = *scan_ptr;
        // Compute what the predicted point cloud would be, if the car was at the pose
        // loc, angle, with the sensor characteristics defined by the provided
        // parameters.
        // This is NOT the motion model predict step: it is the prediction of the
        // expected observations, to be used for the update step.
        const double angle_increment = (angle_max - angle_min) / num_ranges; // Angle increment for the Lidar sensor
        // Note: The returned values must be set using the `scan` variable:
        scan.resize(num_ranges);
        // Fill in the entries of scan using array writes, e.g. scan[i] = ...
        for (size_t i = 0; i < scan.size(); ++i) {
            double min_d = range_max;
//            scan[i] = Vector2f(0, 0);

            double delta_x = range_max * cos(range_min + (angle_increment * i) + angle);
            double delta_y = range_max * sin(range_min + (angle_increment * i) + angle);
            Vector2f laser_point = {loc.x() + delta_x, loc.y() + delta_y};

            // You can create a new line segment instance as follows, for :
            line2f laser_line(loc.x(), loc.y(), laser_point.x(), laser_point.y()); // Line segment from (1,2) to (3.4).
            // Access the end points using `.p0` and `.p1` members:
            printf("P0: %f, %f P1: %f,%f\n",
                   laser_line.p0.x(),
                   laser_line.p0.y(),
                   laser_line.p1.x(),
                   laser_line.p1.y());
            // The line segments in the map are stored in the `map_.lines` variable. You
            // can iterate through them as:
            for (size_t j = 0; j < map_.lines.size(); ++j) {
                const line2f map_line = map_.lines[j];
                // The line2f class has helper functions that will be useful.
                // Check for intersections:
                bool intersects = map_line.Intersects(laser_line);
                // You can also simultaneously check for intersection, and return the point
                // of intersection:
                Vector2f intersection_point; // Return variable
                intersects = map_line.Intersection(laser_line, &intersection_point);
                if (intersects) {
                    double intersection_distance = (intersection_point - loc).norm();
                    printf("Intersects for (%d, %d) at %f,%f. The distance is %f. \n",
                           int(i), int(j),
                           intersection_point.x(),
                           intersection_point.y(),
                           intersection_distance);
                    if (intersection_distance < min_d && intersection_distance > range_min) {
                        // Have intersection between range_min and range_max
                        min_d = intersection_distance; // Update the minimum intersection
                        scan[i] = intersection_point;
                    } else if (intersection_distance < min_d && intersection_distance < range_min) {
                        // Have intersection but is less than range_min
                        printf("Something is blocking the sensor!");
                    }
                } else {
                    scan[i] = laser_point;
                    printf("No intersection for (%d, %d).\n", int(i), int(j));
                }
            }
        }
    }

    void ParticleFilter::Update(const vector<float>& ranges,
                                float range_min,
                                float range_max,
                                float angle_min,
                                float angle_max,
                                Particle* p_ptr) {
        // Implement the update step of the particle filter here.
        // You will have to use the `GetPredictedPointCloud` to predict the expected
        // observations for each particle, and assign weights to the particles based
        // on the observation likelihood computed by relating the observation to the
        // predicted point cloud.
        vector<Vector2f> scan_ptr;
        GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(), range_min, range_max, angle_min, angle_max, &scan_ptr);

        for (size_t i = 0; i < scan_ptr.size(); ++i) {
            double range_est = (scan_ptr[i] - p_ptr->loc).norm();

        }
    }

    void ParticleFilter::Resample() {
        // Resample the particles, proportional to their weights.
        // The current particles are in the `particles_` variable.
        // Create a variable to store the new particles, and when done, replace the
        // old set of particles:
        // vector<Particle> new_particles';
        // During resampling:
        //    new_particles.push_back(...)
        // After resampling:
        // particles_ = new_particles;

        // You will need to use the uniform random number generator provided. For
        // example, to generate a random number between 0 and 1:
        float x = rng_.UniformRandom(0, 1);
        printf("Random number drawn from uniform distribution between 0 and 1: %f\n",
               x);
    }

    void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                      float range_min,
                                      float range_max,
                                      float angle_min,
                                      float angle_max) {
        // A new laser scan observation is available (in the laser frame)
        // Call the Update and Resample steps as necessary.

        // Update particle weight
        for (size_t i = 0; i < FLAGS_num_particles; ++i) {
            Update(ranges, range_min, range_max, angle_min, angle_max, &(particles_[i]));
        }
        // Normalize weight

        // Resample
        Resample();
    }

    void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                         const float odom_angle) {
        // A new odometry value is available (in the odom frame)
        // Implement the motion model predict step here, to propagate the particles
        // forward based on odometry.

        // If the previous odometry data is used
        if (odom_initialized_){
            Vector2f translation_noise;
            double angle_noise;
            // Theta_1^odom is the angle in the odom frame, which is prev_odom_angle_
            Eigen::Rotation2Df rotate_odom_prev(-prev_odom_angle_);
            Vector2f delta_T = rotate_odom_prev * (odom_loc - prev_odom_loc_);
            for (size_t i= 0; i < FLAGS_num_particles; i++) {
                // Generate noise
                translation_noise.x() = rng_.Gaussian(0.0f, 0.1f);
                translation_noise.y() = rng_.Gaussian(0.0f, 0.1f);
                angle_noise = rng_.Gaussian(0.0f, 0.01f);
                // Theta_1^map is the angle in the map frame, which is particles_[i].angle
                Eigen::Rotation2Df rotate_map_prev(particles_[i].angle);
                particles_[i].loc += rotate_map_prev * delta_T + translation_noise;
                particles_[i].angle += odom_angle - prev_odom_angle_ + angle_noise;
            }
            prev_odom_loc_ = odom_loc;
            prev_odom_angle_ = odom_angle;
        }else {
            // If the previous odometry data is not used
            prev_odom_loc_ = odom_loc;
            prev_odom_angle_ = odom_angle;
            odom_initialized_ = true;
        }
    }

    void ParticleFilter::Initialize(const string& map_file,
                                    const Vector2f& loc,
                                    const float angle) {
        // The "set_pose" button on the GUI was clicked, or an initialization message
        // was received from the log. Initialize the particles accordingly, e.g. with
        // some distribution around the provided location and angle.
        map_.Load(map_file);
        double x_noise;
        double y_noise;
        double angle_noise;
        for (size_t i = 0; i < FLAGS_num_particles; ++i) {
            x_noise = rng_.Gaussian(0.0f, 0.1f);
            y_noise = rng_.Gaussian(0.0f, 0.1f);
            angle_noise = rng_.Gaussian(0.0f, 0.01f);
            particles_[i].loc.x() = loc.x() + x_noise;
            particles_[i].loc.y() = loc.y() + y_noise;
            particles_[i].angle = angle + angle_noise;
            particles_[i].weight = 1.0f;
        }
        odom_initialized_ = false;
    }

    void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr,
                                     float* angle_ptr) const {
        Vector2f& loc = *loc_ptr;
        float& angle = *angle_ptr;
        // Compute the best estimate of the robot's location based on the current set
        // of particles. The computed values must be set to the `loc` and `angle`
        // variables to return them. Modify the following assignments:
        double loc_x = 0.0f;
        double loc_y = 0.0f;
        double ang = 0.0f;
        double weight = 0.0f;
        for (size_t i = 0; i < FLAGS_num_particles; ++i) {
            weight = particles_[i].weight;
            loc_x += particles_[i].loc.x() * weight;
            loc_y += particles_[i].loc.y() * weight;
            ang += particles_[i].angle * weight;
        }
        loc = Vector2f(loc_x, loc_y);
        angle = ang;
    }


}  // namespace particle_filter