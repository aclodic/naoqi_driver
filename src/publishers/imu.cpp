/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "imu.hpp"

namespace alros {

namespace publisher {

  ImuPublisher::ImuPublisher(const std::string& topic):
    BasePublisher( topic )
  {}

  void ImuPublisher::publish( const sensor_msgs::Imu& imu_msg)
  {
    pub_.publish(imu_msg);
  }

  void ImuPublisher::reset( ros::NodeHandle& nh )
  {
    pub_ = nh.advertise<sensor_msgs::Imu>(topic_,5);
    is_initialized_ = true;
  }
}

}