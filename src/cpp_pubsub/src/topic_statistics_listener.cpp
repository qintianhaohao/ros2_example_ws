#include <sstream>
#include <string>

#include "topic_statistics_demo/topic_statistics_listener.hpp"

using statistics_msgs::msg::MetricsMessage;
const char * STATISTIC_TYPES[] = {"unknown", "avg", "min", "max", "std_dev", "sample_count"};
bool istimeout_ = false;

TopicStatisticsListener::TopicStatisticsListener(const std::string & topic_name)
: Node("statistics_listener"),
  topic_name_(topic_name) {}

void TopicStatisticsListener::initialize()
{
  RCLCPP_INFO(get_logger(), "TopicStatisticsListener starting up");
  start_listening();
}

void TopicStatisticsListener::start_listening()
{
  if (!subscription_) {
    subscription_ = create_subscription<statistics_msgs::msg::MetricsMessage>(
      topic_name_,
      10,  /* QoS history_depth */
      [this](const typename statistics_msgs::msg::MetricsMessage::SharedPtr msg) -> void
      {
        // RCLCPP_INFO(get_logger(), "Statistics heard:\n%s", MetricsMessageToString(*msg).c_str());
        istimeout_ = isTimeOut(*msg);
      },
      subscription_options_);
  }
}

std::string TopicStatisticsListener::MetricsMessageToString(const MetricsMessage & results)
{
  std::stringstream ss;
  ss << "Metric name: " << results.metrics_source <<
    " source: " << results.measurement_source_name <<
    " unit: " << results.unit;
  ss << "\nWindow start: " << results.window_start.nanosec << " end: " <<
    results.window_stop.nanosec;

  for (const auto & statistic : results.statistics) {
    ss << "\n" <<
      STATISTIC_TYPES[statistic.data_type] <<
      ": " <<
      std::to_string(statistic.data);
  }

  return ss.str();
}

bool TopicStatisticsListener::isTimeOut(const MetricsMessage & results)
{
  for (const auto & statistic : results.statistics) {
    if (statistic.data_type == 1 && statistic.data > 2000) {
      RCLCPP_WARN(get_logger(), "msg time out! average time: %f", statistic.data);
      return true;
    }
  }
  return false;
}