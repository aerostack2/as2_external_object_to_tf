/*!*******************************************************************************************
 *  \file       as2_external_object_to_tf.cpp
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

#include "as2_external_object_to_tf.hpp"
#include "yaml-cpp/yaml.h"

As2ExternalObjectToTf::As2ExternalObjectToTf() : as2::Node("external_object_to_tf") {
  this->declare_parameter("config_file", "config/external_objects.yaml");
  this->get_parameter("config_file", config_path_);
}

auto pose_callback_factory = [](const std::string& frame_id, const std::string& parent_frame_id) {
  return
      [frame_id, parent_frame_id](const geometry_msgs::msg::PoseStamped::SharedPtr _msg) -> void {
        geometry_msgs::msg::TransformStamped transform;
        transform.child_frame_id          = frame_id;
        transform.header.frame_id         = parent_frame_id;
        transform.header.stamp            = _msg->header.stamp;
        transform.transform.rotation      = _msg->pose.orientation;
        transform.transform.translation.x = _msg->pose.position.x;
        transform.transform.translation.y = _msg->pose.position.y;
        transform.transform.translation.z = _msg->pose.position.z;
        As2ExternalObjectToTf::tfBroadcaster->sendTransform(transform);
      };
};

auto gps_callback_factory = [](const std::string& frame_id, const std::string& parent_frame_id) {
  return [frame_id, parent_frame_id](const sensor_msgs::msg::NavSatFix::SharedPtr _msg) -> void {
    As2ExternalObjectToTf::gps_poses[frame_id].gps_pose = _msg;
    if (As2ExternalObjectToTf::gps_poses[frame_id].azimuth != NULL) {
      As2ExternalObjectToTf::tfBroadcaster->sendTransform(As2ExternalObjectToTf::gpsToTransform(
          As2ExternalObjectToTf::gps_poses[frame_id].gps_pose,
          As2ExternalObjectToTf::gps_poses[frame_id].azimuth, frame_id, parent_frame_id));
    }
  };
};

auto azimuth_callback_factory = [](const std::string& frame_id,
                                   const std::string& parent_frame_id) {
  return [frame_id, parent_frame_id](const std_msgs::msg::Float32::SharedPtr _msg) -> void {
    As2ExternalObjectToTf::gps_poses[frame_id].azimuth = _msg;
  };
};

geometry_msgs::msg::Quaternion As2ExternalObjectToTf::azimuthToQuaternion(
    std_msgs::msg::Float32::SharedPtr azimuth) {
  float azimuthRad = azimuth->data * M_PI / 180.0;
  azimuthRad += M_PI / 2;
  if (azimuthRad > M_PI) {
    azimuthRad -= (2 * M_PI);
  }
  geometry_msgs::msg::Quaternion q;
  double halfYaw = azimuthRad * 0.5;
  q.x            = 0.0;
  q.y            = 0.0;
  q.z            = cos(halfYaw);
  q.w            = sin(halfYaw);
  return q;
}

geometry_msgs::msg::TransformStamped As2ExternalObjectToTf::gpsToTransform(
    const sensor_msgs::msg::NavSatFix::SharedPtr gps_pose,
    const std_msgs::msg::Float32::SharedPtr azimuth,
    const std::string frame_id,
    const std::string parent_frame_id) {
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp              = gps_pose->header.stamp;
  transform.header.frame_id           = parent_frame_id;
  transform.child_frame_id            = frame_id;
  sensor_msgs::msg::NavSatFix* rawPtr = gps_pose.get();
  gps_handler->LatLon2Local(*rawPtr, transform.transform.translation.x,
                            transform.transform.translation.y, transform.transform.translation.z);
  transform.transform.rotation = azimuthToQuaternion(azimuth);
  return transform;
}

void As2ExternalObjectToTf::loadObjects(const std::string path) {
  try {
    YAML::Node config = YAML::LoadFile(config_path_);
    auto objects      = config["objects"];
    for (YAML::const_iterator object = objects.begin(); object != objects.end(); ++object) {
      std::string type = (*object)["frame"].as<std::string>();

      if ((*object)["type"].as<std::string>() == "pose") {
        std::string parent_frame = ((*object)["parent_frame"].IsDefined())
                                       ? (*object)["parent_frame"].as<std::string>()
                                       : "earth";

        pose_subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
            (*object)["pose_topic"].as<std::string>(), as2_names::topics::self_localization::qos,
            pose_callback_factory((*object)["frame"].as<std::string>(), parent_frame)));

      } else if ((*object)["type"].as<std::string>() == "gps") {
        if (!origin_set_) {
          setupGPS();
          origin_set_ = true;
        }
        As2ExternalObjectToTf::gps_poses[(*object)["frame"].as<std::string>()] = gps_object();
        std::string parent_frame = ((*object)["parent_frame"].IsDefined())
                                       ? (*object)["parent_frame"].as<std::string>()
                                       : "earth";
        gps_subs_.push_back(this->create_subscription<sensor_msgs::msg::NavSatFix>(
            (*object)["gps_topic"].as<std::string>(), as2_names::topics::self_localization::qos,
            gps_callback_factory((*object)["frame"].as<std::string>(), parent_frame)));
        azimuth_subs_.push_back(this->create_subscription<std_msgs::msg::Float32>(
            (*object)["azimuth_topic"].as<std::string>(), as2_names::topics::self_localization::qos,
            azimuth_callback_factory((*object)["frame"].as<std::string>(), parent_frame)));

      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid type for object '%s', types are: pose or gps",
                    type.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "Object '%s' Succesfully loaded from config file",
                  type.c_str());
    }
  } catch (YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "YAML error: %s", e.what());
  }
}

void As2ExternalObjectToTf::setupNode() {
  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  loadObjects(config_path_);
}

void As2ExternalObjectToTf::setupGPS() {
  get_origin_srv_ = this->create_client<as2_msgs::srv::GetOrigin>(
      as2_names::services::gps::get_origin);  // Should be same origin for every drone ?

  while (!get_origin_srv_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<as2_msgs::srv::GetOrigin::Request>();

  request->structure_needs_at_least_one_member = 0;

  auto result = get_origin_srv_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result,
                                         std::chrono::seconds(1)) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    // ;

  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service get origin");
    return;
  }
  origin_ = std::make_unique<geographic_msgs::msg::GeoPoint>(result.get()->origin);
  RCLCPP_INFO(this->get_logger(), "Origin in: lat: %f, lon %f, alt: %f", origin_->latitude,
              origin_->longitude, origin_->altitude);
  gps_handler = std::make_unique<as2::gps::GpsHandler>(origin_->latitude, origin_->longitude,
                                                       origin_->altitude);
}

std::unique_ptr<tf2_ros::TransformBroadcaster> As2ExternalObjectToTf::tfBroadcaster = NULL;
std::unique_ptr<as2::gps::GpsHandler> As2ExternalObjectToTf::gps_handler            = NULL;
std::map<std::string, gps_object> As2ExternalObjectToTf::gps_poses;

void As2ExternalObjectToTf::run() { return; }

void As2ExternalObjectToTf::cleanupNode() { return; }

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn As2ExternalObjectToTf::on_configure(const rclcpp_lifecycle::State& _state) {
  // Set subscriptions, publishers, services, actions, etc. here.
  setupNode();

  return CallbackReturn::SUCCESS;
};

CallbackReturn As2ExternalObjectToTf::on_deactivate(const rclcpp_lifecycle::State& _state) {
  // Clean up subscriptions, publishers, services, actions, etc. here.
  return CallbackReturn::SUCCESS;
};

CallbackReturn As2ExternalObjectToTf::on_shutdown(const rclcpp_lifecycle::State& _state) {
  // Clean other resources here.

  return CallbackReturn::SUCCESS;
};
