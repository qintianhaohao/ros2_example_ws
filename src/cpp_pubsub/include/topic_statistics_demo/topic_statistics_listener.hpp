#ifndef TOPIC_STATISTICS_DEMO__TOPIC_STATISTICS_LISTENER_HPP_
#define TOPIC_STATISTICS_DEMO__TOPIC_STATISTICS_LISTENER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "statistics_msgs/msg/metrics_message.hpp"

namespace topic_stats_demo
{
constexpr char STATISTICS_TOPIC_NAME[] = "statistics";
}

class TopicStatisticsListener : public rclcpp::Node
{
public:
  /// Standard Constructor.
  /**
    * \param[in] topic_name Topic to which statistics are published.
    */
  explicit TopicStatisticsListener(
    const std::string & topic_name = topic_stats_demo::STATISTICS_TOPIC_NAME);

  /// Initialize the listener node.
  void initialize();

  /// Return string representation of a MetricsMessage.
  /**
    * \param[in] results Statistics heard form the subscribed topic.
    * \param[out] String representation of the input statistics.
    */
  std::string MetricsMessageToString(const statistics_msgs::msg::MetricsMessage & results);

  /// Instantiate a Subscription to the statistics topic.
  void start_listening();

  /// return true if average time in statistic over 2s
  bool isTimeOut(const statistics_msgs::msg::MetricsMessage & results);

private:
  rclcpp::SubscriptionOptions subscription_options_;
  rclcpp::Subscription<statistics_msgs::msg::MetricsMessage>::SharedPtr subscription_ = nullptr;

  const std::string topic_name_;
};

#endif  // TOPIC_STATISTICS_DEMO__TOPIC_STATISTICS_LISTENER_HPP_
