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

#include "c1t_rplidar_driver/c1t_rplidar_driver.h"

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

RplidarDriverWrapper::RplidarDriverWrapper(int argc, char** argv, const std::string& name)
  : cav::DriverWrapper(argc, argv, name), lidar_timeout_(0.5)
{
}

void RplidarDriverWrapper::initialize()
{
  status_.lidar = true;
  point_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>("points_raw", 1);

  scan_sub_ =
      nh_->subscribe<sensor_msgs::LaserScan>("scan", 1, [this](const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        last_update_time_ = ros::Time::now();
        status_.status = cav_msgs::DriverStatus::OPERATIONAL;

        sensor_msgs::PointCloud2 pc_msg;
        projector_.projectLaser(*scan_msg, pc_msg);

        point_cloud_pub_.publish(pc_msg);
      });

  private_nh_->param<double>("lidar_timeout", lidar_timeout_, 0.5);
}

void RplidarDriverWrapper::pre_spin()
{
  this->checkLidarTimeout();
}

void RplidarDriverWrapper::post_spin()
{
}

void RplidarDriverWrapper::shutdown()
{
}

void RplidarDriverWrapper::checkLidarTimeout()
{
  if (last_update_time_.isZero() || ros::Time::now() - last_update_time_ > ros::Duration(lidar_timeout_))
  {
    status_.status = cav_msgs::DriverStatus::OFF;
  }
}
