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
#include "json.hpp"

As2ExternalObjectToTf::As2ExternalObjectToTf() : as2::Node("external_object_to_tf") {
  this->declare_parameter("config_file", "config/external_objects.json");
  this->get_parameter("config_file", config_path_);
}

std::unique_ptr<tf2_ros::TransformBroadcaster> As2ExternalObjectToTf::tfBroadcaster = NULL;

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
    printf("frame_name: %s, parent_frame: %s, data: %f\n", frame_id.c_str(),
           parent_frame_id.c_str(), _msg->altitude);
  };
};

auto azimuth_callback_factory = [](const std::string& frame_id,
                                   const std::string& parent_frame_id) {
  return [frame_id, parent_frame_id](const std_msgs::msg::Float32::SharedPtr _msg) -> void {
    printf("frame_name: %s, parent_frame: %s, data: %f\n", frame_id.c_str(),
           parent_frame_id.c_str(), _msg->data);
  };
};

void As2ExternalObjectToTf::loadObjects(const std::string path) {
  std::ifstream fJson(path);
  std::stringstream buffer;
  buffer << fJson.rdbuf();
  auto json = nlohmann::json::parse(buffer.str());
  for (auto json_geofence : json["objects"]) {
    // if (!checkValidity(std::size(json_geofence["polygon"]), json_geofence["id"],
    //                    json_geofence["type"])) {
    //   return;
    // } else {

    std::string type = json_geofence["frame"];
    try {
      if (json_geofence["type"] == "pose") {
        std::string parent_frame =
            (!json_geofence["parent_frame"].is_null()) ? json_geofence["parent_frame"] : "earth";

        pose_subs_.push_back(this->create_subscription<geometry_msgs::msg::PoseStamped>(
            json_geofence["pose_topic"], as2_names::topics::self_localization::qos,
            pose_callback_factory(json_geofence["frame"], parent_frame)));

      } else if (json_geofence["type"] == "gps") {
        if (!origin_set_) {
          setupGPS();
          origin_set_ = true;
        }
        std::string parent_frame =
            (!json_geofence["parent_frame"].is_null()) ? json_geofence["parent_frame"] : "earth";
        gps_subs_.push_back(this->create_subscription<sensor_msgs::msg::NavSatFix>(
            json_geofence["gps_topic"], as2_names::topics::self_localization::qos,
            gps_callback_factory(json_geofence["frame"], parent_frame)));
        azimuth_subs_.push_back(this->create_subscription<std_msgs::msg::Float32>(
            json_geofence["azimuth_topic"], as2_names::topics::self_localization::qos,
            azimuth_callback_factory(json_geofence["frame"], parent_frame)));

      } else {
        RCLCPP_WARN(this->get_logger(), "Invalid type for object '%s', types are: pose or gps",
                    type.c_str());
      }
      RCLCPP_INFO(this->get_logger(), "Object '%s' Succesfully loaded from JSON file",
                  type.c_str());

    } catch (nlohmann::json::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Json error: %s", e.what());
    }
  }
}

void As2ExternalObjectToTf::setupNode() {
  tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  loadObjects(config_path_);
}

void As2ExternalObjectToTf::setupGPS() {
  get_origin_srv_ = this->create_client<as2_msgs::srv::GetOrigin>(
      as2_names::services::gps::get_origin);  // Should be same origin for every drone ?
  auto request = std::make_shared<as2_msgs::srv::GetOrigin::Request>();

  while (!get_origin_srv_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto result = get_origin_srv_->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    *origin_ = result.get()->origin;
    RCLCPP_INFO(this->get_logger(), "Origin in: lat: %f, lon %f, alt: %f",
                result.get()->origin.latitude, result.get()->origin.longitude,
                result.get()->origin.altitude);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service get origin");
  }
  gps_handler = std::make_unique<as2::gps::GpsHandler>(origin_->latitude, origin_->longitude,
                                                       origin_->longitude);
}

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
