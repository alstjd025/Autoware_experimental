// Copyright 2019 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <memory>
#include <string>

#include "controller_common_nodes/controller_base_node.hpp"

int32_t main(const int32_t argc, char * argv[])
{
  int32_t ret = 0;
  try {
    rclcpp::init(argc, argv);
    using motion::control::controller_common_nodes::ControllerBaseNode;
    const auto nd_ptr = std::make_shared<ControllerBaseNode>("controller_common_node", "control");

    rclcpp::spin(nd_ptr);

    if (!rclcpp::shutdown()) {
      throw std::runtime_error{"Could not shutdown rclcpp"};
    }
  } catch (const std::exception & e) {
    std::cerr << e.what();
    ret = __LINE__;
  } catch (...) {
    std::cerr << "Unknown error occured";
    ret = __LINE__;
  }

  return ret;
}
