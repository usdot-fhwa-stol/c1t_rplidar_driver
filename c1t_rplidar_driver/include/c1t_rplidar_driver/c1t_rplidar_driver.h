#ifndef C1T_RPLIDAR_DRIVER_H
#define C1T_RPLIDAR_DRIVER_H

/**
 * Copyright 2021 U.S. Department of Transportation, Federal Highway Administration
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cav_driver_utils/driver_wrapper/driver_wrapper.h>
#include <laser_geometry/laser_geometry.h>

/**
 * Slamtec RPLIDAR ROS package wrapper that conforms to the CARMA lidar hardware interface.
 * 
 * Subscribers:
 *     system_alert: CARMA system_alert topic
 *     scan: LaserScan messages from the lidar
 * 
 * Publishers:
 *     driver_discovery: driver information
 *     points_raw: laser scan data converted to PointCloud2 messages
 */
class RplidarDriverWrapper : public cav::DriverWrapper
{
public:
  /**
   * @brief RplidarDriverWrapper constructor
   * 
   * @param argc commandline argument count
   * @param argv array of commandline argument values
   * @param name ROS node name
   */
  RplidarDriverWrapper(int argc, char** argv, const std::string& name = "c1t_rplidar_driver");

  /**
   * @brief RplidarDriverWrapper destructor
   */
  virtual ~RplidarDriverWrapper() = default;

private:
  ros::Subscriber scan_sub_;
  ros::Publisher point_cloud_pub_;
  ros::Time last_update_time_;
  double lidar_timeout_;

  laser_geometry::LaserProjection projector_;

  /**
   * @brief Initializes the ROS node
   * 
   * This function is called before the ROS node starts running.
   */
  void initialize() override;

  /**
   * @brief Process stuff before spinning
   * 
   * This function is called before the spinOnce() function.
   */
  void pre_spin() override;

  /**
   * @brief Process stuff after spinning
   * 
   * This function is called after the spinOnce() function.
   */
  void post_spin() override;

  /**
   * @brief Prepare ROS node for shutting down
   * 
   * This function is called before the node is shut down.
   */
  void shutdown() override;

  /**
   * @brief Check if the lidar has timed out
   * 
   * Check if the time between lidar messages exceeds the timeout threshold.
   */
  void checkLidarTimeout();
};

#endif  // C1T_RPLIDAR_DRIVER_H