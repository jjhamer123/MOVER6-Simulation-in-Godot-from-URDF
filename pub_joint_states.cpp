#ifndef GODOT__GODOT_ROS__DEMOS__JointStatePublisher_HPP
#define GODOT__GODOT_ROS__DEMOS__JointStatePublisher_HPP

#include <iostream>

#include "core/object/ref_counted.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatePublisher : public RefCounted {
  GDCLASS(JointStatePublisher, RefCounted);
public:
  JointStatePublisher() {
	try {
	  rclcpp::init(0, nullptr);
	} catch (...) {
	  std::cout << "Already initialized, ignoring..." << std::endl;
	}

	m_node = std::make_shared<rclcpp::Node>("godot_JointStatePublisher_node");
	publisher_ = m_node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  }

  ~JointStatePublisher() {
	rclcpp::shutdown();
  }

  inline void spin_some() {
	rclcpp::spin_some(m_node);
  }

  // joint_names, positions, velocities must all be the same length, matched by index
  void publish_joint_state(PackedStringArray joint_names, PackedFloat64Array positions, PackedFloat64Array velocities)
  {
	sensor_msgs::msg::JointState msg;
	msg.header.stamp = m_node->get_clock()->now();
	for (int i = 0; i < joint_names.size(); ++i) {
	  msg.name.push_back(std::string(joint_names[i].utf8().get_data()));
	}
	for (int i = 0; i < positions.size(); ++i) {
	  msg.position.push_back(positions[i]);
	}
	for (int i = 0; i < velocities.size(); ++i) {
	  msg.velocity.push_back(velocities[i]);
	}
	publisher_->publish(msg);
  }

protected:
  static void _bind_methods();

  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
};
#endif
