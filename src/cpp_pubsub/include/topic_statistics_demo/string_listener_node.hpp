#ifndef TOPIC_STATISTICS_DEMO__STRING_LISTENER_NODES_HPP_
#define TOPIC_STATISTICS_DEMO__STRING_LISTENER_NODES_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StringListener : public rclcpp::Node
{
public:
  /// Standard constructor.
  /**
    * \param[in] topic_name Topic to subscribe to.
    * \param[in] subscription_options SubscriptionOptions to use for the subscription.
    */
  StringListener(
    const std::string & topic_name = "string_chatter",
    const rclcpp::SubscriptionOptions & subscription_options = rclcpp::SubscriptionOptions());

  /// Initialize the listener node.
  void initialize();

  /// Instantiate a Subscription to the chatter topic.
  void start_listening();

private:
  rclcpp::SubscriptionOptions subscription_options_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ = nullptr;

  const std::string topic_name_;
};

#endif  // TOPIC_STATISTICS_DEMO__STRING_TALKER_LISTENER_NODES_HPP_
