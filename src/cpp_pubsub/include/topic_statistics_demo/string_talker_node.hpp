#ifndef TOPIC_STATISTICS_DEMO__STRING_TALKER_NODES_HPP_
#define TOPIC_STATISTICS_DEMO__STRING_TALKER_NODES_HPP_

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class StringTalker : public rclcpp::Node
{
public:
  /// Standard constructor.
  /**
    * \param[in] topic_name Topic to publish chatter messages to.
    * \param[in] publish_period Frequency of publishing chatter messages.
    **/
  StringTalker(
    const std::string & topic_name = "string_chatter",
    std::chrono::milliseconds publish_period = std::chrono::milliseconds(3000));

  /// Initialize the publisher.
  void initialize();

  /// Publish a single message.
  void publish();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ = nullptr;

  const std::string topic_name_;

  std::chrono::milliseconds publish_period_ = std::chrono::milliseconds(1000);
  rclcpp::TimerBase::SharedPtr publish_timer_ = nullptr;
  size_t publish_count_ = 0;
};

#endif
