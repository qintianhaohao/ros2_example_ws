#include <chrono>
#include <string>

#include "topic_statistics_demo/string_talker_node.hpp"

using namespace std::chrono_literals;

StringTalker::StringTalker(
  const std::string & topic_name,
  std::chrono::milliseconds publish_period)
: Node("string_talker"),
  topic_name_(topic_name),
  publish_period_(publish_period) {}

void StringTalker::initialize()
{
  RCLCPP_INFO(get_logger(), "Talker starting up");

  publisher_ = this->create_publisher<std_msgs::msg::String>(
    topic_name_,
    10 /* QoS history_depth */);
  publish_timer_ = create_wall_timer(
    publish_period_,
    [this]() -> void {
      publish();
    });
}

void StringTalker::publish()
{
  std_msgs::msg::String msg;
  msg.data = "Talker says " + std::to_string(publish_count_);
  RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
  publisher_->publish(msg);

  ++publish_count_;
}