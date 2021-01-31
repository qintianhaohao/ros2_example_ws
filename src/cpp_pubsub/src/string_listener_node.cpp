#include <chrono>
#include <string>

#include "topic_statistics_demo/string_listener_node.hpp"

using namespace std::chrono_literals;

StringListener::StringListener(
  const std::string & topic_name,
  const rclcpp::SubscriptionOptions & subscription_options)
: Node("string_listener"),
  subscription_options_(subscription_options),
  topic_name_(topic_name) {}

void StringListener::initialize()
{
  RCLCPP_INFO(get_logger(), "Listener starting up");
  start_listening();
}

void StringListener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<std_msgs::msg::String>(
      topic_name_,
      10,  /**QoS history_depth */
      [this](const typename std_msgs::msg::String::SharedPtr msg) -> void
      {
        RCLCPP_INFO(get_logger(), "Listener heard: [%s]", msg->data.c_str());
      },
      subscription_options_);
  }
}
