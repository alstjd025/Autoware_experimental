// Copyright 2020-2021 Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware_auto_planning_msgs/srv/modify_trajectory.hpp>
#include <autoware_auto_tf2/tf2_autoware_auto_msgs.hpp>
#include <motion_common/config.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <common/types.hpp>
#include <string>
#include <memory>

#include "object_collision_estimator_nodes/object_collision_estimator_node.hpp"
#include "object_collision_estimator_nodes/visualize.hpp"
#include "object_collision_estimator/object_collision_estimator.hpp"

#include "motion_common/motion_common.hpp"

namespace motion
{
namespace planning
{
namespace object_collision_estimator_nodes
{

using namespace std::chrono_literals;
using motion::planning::object_collision_estimator::ObjectCollisionEstimatorConfig;
using motion::planning::object_collision_estimator::ObjectCollisionEstimator;
using motion::planning::trajectory_smoother::TrajectorySmootherConfig;
using motion::planning::trajectory_smoother::TrajectorySmoother;
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using motion::motion_common::VehicleConfig;
using motion::motion_common::Real;
using autoware::common::types::float64_t;
using autoware::common::types::float32_t;
using rclcpp::QoS;

ObjectCollisionEstimatorNode::ObjectCollisionEstimatorNode(const rclcpp::NodeOptions & node_options)
: Node(OBJECT_COLLISION_ESTIMATOR_NODE_NAME, node_options)
{
  // Declare node parameters. See ObjectCollisionEstimator Class for details of the functions of
  // these parameters.
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter(
      "vehicle.cg_to_front_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.cg_to_rear_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.front_corner_stiffness"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.rear_corner_stiffness"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.mass_kg"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.yaw_inertia_kgm2"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.width_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.front_overhang_m"
    ).get<float32_t>()),
    static_cast<Real>(declare_parameter(
      "vehicle.rear_overhang_m"
    ).get<float32_t>())
  };

  const auto safety_factor =
    static_cast<float32_t>(declare_parameter(
      "safety_factor"
    ).get<float32_t>());
  const auto stop_margin =
    static_cast<float32_t>(declare_parameter(
      "stop_margin"
    ).get<float32_t>());
  const auto min_obstacle_dimension_m =
    static_cast<float32_t>(declare_parameter(
      "min_obstacle_dimension_m"
    ).get<float32_t>());
  const TrajectorySmootherConfig smoother_config {
    static_cast<float32_t>(declare_parameter(
      "trajectory_smoother.kernel_std"
    ).get<float32_t>()),
    static_cast<uint32_t>(declare_parameter(
      "trajectory_smoother.kernel_size"
    ).get<uint32_t>())
  };

  // Object staleness time threshold
  m_staleness_threshold_ms = std::chrono::milliseconds(
    static_cast<uint32_t>(declare_parameter(
      "staleness_threshold_ms"
    ).get<uint32_t>())
  );

  // the tf frame in which planned local trajectories are published
  m_target_frame_id =
    static_cast<std::string>(declare_parameter(
      "target_frame_id"
    ).get<std::string>());

  // Create an object collision estimator
  const ObjectCollisionEstimatorConfig config {vehicle_param, safety_factor, stop_margin,
    min_obstacle_dimension_m};
  const TrajectorySmoother smoother{smoother_config};
  m_estimator = std::make_unique<ObjectCollisionEstimator>(config, smoother);

  // Set up service interface for collision_detection
  m_service_interface = create_service<autoware_auto_planning_msgs::srv::ModifyTrajectory>(
    "estimate_collision",
    [this](const std::shared_ptr<autoware_auto_planning_msgs::srv::ModifyTrajectory::Request>
    request,
    std::shared_ptr<autoware_auto_planning_msgs::srv::ModifyTrajectory::Response> response) {
      estimate_collision(request, response);
    });

  m_predicted_objects_sub = Node::create_subscription<PredictedObjects>(
    "predicted_objects", QoS{10},
    [this](const PredictedObjects::SharedPtr msg) {this->on_predicted_object(msg);});

  m_trajectory_bbox_pub =
    create_publisher<MarkerArray>("debug/trajectory_bounding_boxes", QoS{10});

  // Create a tf interface to perform transforms on obstacle bounding boxes
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(
    *m_tf_buffer,
    std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);
  m_tf_buffer->setUsingDedicatedThread(true);
}

void ObjectCollisionEstimatorNode::on_predicted_object(const PredictedObjects::SharedPtr & msg)
{
  if (msg->header.frame_id != m_target_frame_id) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Target frame should be same with Tracking/Prediction frame");
  }
  m_last_obstacle_msg_time = msg->header.stamp;
  m_estimator->updatePredictedObjects(*msg);
}

void ObjectCollisionEstimatorNode::estimate_collision(
  const std::shared_ptr<autoware_auto_planning_msgs::srv::ModifyTrajectory::Request> request,
  std::shared_ptr<autoware_auto_planning_msgs::srv::ModifyTrajectory::Response> response)
{
  rclcpp::Time request_time{
    request->original_trajectory.header.stamp,
    m_last_obstacle_msg_time.get_clock_type()};

  auto elapsed_time = request_time - m_last_obstacle_msg_time;
  if (m_last_obstacle_msg_time.seconds() == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "No obstacle information has been received. Collision estimation will have no effect");
  } else if (elapsed_time > m_staleness_threshold_ms) {
    RCLCPP_WARN(
      this->get_logger(),
      "Outdated obstacle information."
      "Collision estimation will be based on old obstacle positions");
  }

  // copy the input trajectory into the output variable
  response->modified_trajectory = request->original_trajectory;
  Trajectory trajectory_transformed = request->original_trajectory;

  const tf2::TimePoint trajectory_time_point = tf2::TimePoint(
    std::chrono::seconds(request->original_trajectory.header.stamp.sec) +
    std::chrono::nanoseconds(request->original_trajectory.header.stamp.nanosec));

  if (response->modified_trajectory.header.frame_id != m_target_frame_id) {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = this->m_tf_buffer->lookupTransform(
        m_target_frame_id,
        request->original_trajectory.header.frame_id,
        trajectory_time_point);
    } catch (const tf2::ExtrapolationException &) {
      tf = this->m_tf_buffer->lookupTransform(
        m_target_frame_id,
        request->original_trajectory.header.frame_id,
        tf2::TimePointZero);
    }

    for (auto & trajectory_point : trajectory_transformed.points) {
      ::motion::motion_common::doTransform(
        trajectory_point, trajectory_point, tf);
    }
  }

  // m_estimator performs the collision estimation and the trajectory will get updated inside
  const auto collision_index = m_estimator->updatePlan(
    trajectory_transformed,
    response->modified_trajectory);
  // publish trajectory bounding box for visualization
  auto trajectory_bbox = m_estimator->getTrajectoryBoundingBox();
  trajectory_bbox.header.frame_id = m_target_frame_id;
  trajectory_bbox.header.stamp = request->original_trajectory.header.stamp;
  auto marker = toVisualizationMarkerArray(
    trajectory_bbox,
    (collision_index <= -1 ) ?
    request->original_trajectory.points.size() : static_cast<size_t>(collision_index));
  m_trajectory_bbox_pub->publish(marker);
}

}  // namespace object_collision_estimator_nodes
}  // namespace planning
}  // namespace motion

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(
  motion::planning::object_collision_estimator_nodes::ObjectCollisionEstimatorNode)
