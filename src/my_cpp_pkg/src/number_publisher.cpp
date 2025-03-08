#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublishNode : public rclcpp::Node
{
public:
  NumberPublishNode():Node("number_publisher")
  {
    this->declare_parameter("number", 2);
    this->declare_parameter("timer_period", 1.0);

    this->number_ = this->get_parameter("number").as_int();
    this->timer_period_ = this->get_parameter("timer_period").as_double();

    this->number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
    this->number_timer_ = this->create_wall_timer(std::chrono::duration<double>(this->timer_period_), 
                                                  std::bind(&NumberPublishNode::publishNumber, this));

    this->param_callback_handle_ = this->add_post_set_parameters_callback(
                      std::bind(&NumberPublishNode::parametersCallback, this, std::placeholders::_1));                                              
    RCLCPP_INFO(this->get_logger(), "Number Publisher has been started");

  }
private:
  void publishNumber(){
    auto msg = example_interfaces::msg::Int64();
    msg.data = this->number_;
    this->number_publisher_->publish(msg);
  }

  void parametersCallback(const std::vector<rclcpp::Parameter> &parameters){
    for(const auto &param: parameters){
      if(param.get_name() == "number"){
        this->number_ = param.as_int();
      }
    }
  }

  int number_;
  double timer_period_;
  rclcpp::TimerBase::SharedPtr number_timer_;
  rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
  PostSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NumberPublishNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}