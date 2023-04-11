/*!*******************************************************************************************
 *  \file       as2_external_object_to_tf.hpp
 *  \brief      External object to TF for Aerostack2
 *  \authors    Javier Melero Deza
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#ifndef AS2_EXTERNAL_OBJECT_TO_TF_HPP_
#define AS2_EXTERNAL_OBJECT_TO_TF_HPP_

#include <fstream>
#include <iostream>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geographic_msgs/msg/geo_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include "as2_core/names/actions.hpp"
#include "as2_core/names/services.hpp"
#include "as2_core/names/topics.hpp"
#include "as2_core/node.hpp"
#include "as2_msgs/srv/get_origin.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float32.hpp"

class As2ExternalObjectToTf : public as2::Node {
public:
  As2ExternalObjectToTf();

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  void setupNode();
  void cleanupNode();
  void run();

  CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State&) override;

  struct pose_object {         // Structure declaration
    std::string parent_frame;  // object parent frame
    std::string frame;         // object frame name
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
  };

  struct gps_object {
    std::string parent_frame;  // object parent frame
    std::string frame;         // object frame name
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr azimuth_sub;
  };

private:
  std::string config_path_;
  std::vector<pose_object> tf_pose_objects_;
  std::vector<gps_object> tf_gps_objects_;
  geographic_msgs::msg::GeoPoint::UniquePtr origin_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      objects_subscriptions_;
  void loadObjects(const std::string path);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr _msg);
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr _msg);
  void azimuthCallback(const std_msgs::msg::Float32::SharedPtr _msg);
  rclcpp::Client<as2_msgs::srv::GetOrigin>::SharedPtr get_origin_srv_;
};

#endif  // AS2_EXTERNAL_OBJECT_TO_TF_HPP_
