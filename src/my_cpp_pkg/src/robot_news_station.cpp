#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewwStationNode : public rclcpp::Node
{
public:
  RobotNewwStationNode():Node("cpp_test"), robot_name_("AA30")
  {
    publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);

    RCLCPP_INFO(this->get_logger(), "HellO, this is AA30");
    timer_ = this->create_wall_timer(std::chrono::seconds(1), 
                                      std::bind(&RobotNewwStationNode::timerCallback, this));
  }
  
private:
  void timerCallback()
  {
    this->publishNews();
  }

  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_name_;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;

  void publishNews()
  {
    auto msg = example_interfaces::msg::String();
    msg.data = "Hi, this is " + this->robot_name_;
    this->publisher_->publish(msg);
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNewwStationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}