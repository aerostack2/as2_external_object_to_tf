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

As2ExternalObjectToTf::As2ExternalObjectToTf() : as2::Node("as2_external_object_to_tf") {
  this->declare_parameter("config_file", "config/external_objects.json");
  this->get_parameter("config_file", config_path_);
}

void As2ExternalObjectToTf::loadObjects(const std::string path) {
  std::ifstream fJson(path);
  std::stringstream buffer;
  buffer << fJson.rdbuf();
  auto json = nlohmann::json::parse(buffer.str());
  object object_to_load;
  for (auto json_geofence : json["objects"]) {
    // if (!checkValidity(std::size(json_geofence["polygon"]), json_geofence["id"],
    //                    json_geofence["type"])) {
    //   return;
    // } else {
    object_to_load.parent_frame =
        (json_geofence["parent_frame"] != NULL) ? json_geofence["parent_frame"] : "earth";
    object_to_load.frame = json_geofence["frame"];
    object_to_load.topic = json_geofence["topic"];
    tf_objects_.push_back(object_to_load);

    RCLCPP_INFO(this->get_logger(), "Object Succesfully loaded from JSON file");
  }
}

void As2ExternalObjectToTf::setupNode() {
  loadObjects(config_path_);

  for (std::vector<object>::iterator ptr = tf_objects_.begin(); ptr < tf_objects_.end(); ptr++) {
    object_callback =
  }

  return;
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

ObjectSubscriptorCallback::ObjectSubscriptorCallback(As2ExternalObjectToTf::object obj) {
  object_ = obj;
}

void ObjectSubscriptorCallback::poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr _msg) {}