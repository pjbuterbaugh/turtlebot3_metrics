/* get_metrics
 *
 * A basic library to grab data from a TurtleBot3.
 *
 * References:
 * - <https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?>
 * -
 * <https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html>
 *
 */

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class get_metrics : public rclcpp::Node {
public:
  get_metrics() : Node("get_metrics") {
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/battery_state", 10,
        std::bind(&get_metrics::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String &msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<get_metrics>());
  rclcpp::shutdown();
  return 0;
}
