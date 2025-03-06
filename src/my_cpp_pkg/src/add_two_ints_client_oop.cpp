#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
  AddTwoIntsClientNode():Node("add_two_ints_client")
  {
    this->client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
  }

  void callAddTwoInts(int a, int b)
  {
    while(not this->client_->wait_for_service(std::chrono::seconds(1))){
      RCLCPP_WARN(this->get_logger(), "Waiting for Add Two Ints server...");
    }
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;
  
    this->client_->async_send_request(request, 
      std::bind(&AddTwoIntsClientNode::callback_add_two_ints, this, std::placeholders::_1));
  }
private:
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

  void callback_add_two_ints(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "result: %d", (int)response->sum);
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClientNode>();
  node->callAddTwoInts(6,100);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}