#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using ROSString = std_msgs::msg::String;

class ROSTopicSubsriber : public rclcpp::Node
{
  static constexpr auto node_name = "ROS_Topic_Subscriber";
  static constexpr auto topic = "ROS";
  static constexpr auto qos = 10;

public:
    ROSTopicSubsriber() : Node(node_name)
    {
      subscription = this->create_subscription<ROSString>(topic, qos, std::bind(&ROSTopicSubsriber::rosTopicCallback,
                                                                                this,
                                                                                std::placeholders::_1));
    }

private:
    void rosTopicCallback(const ROSString & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Llego un mensaje");
      //RCLCPP_INFO(this->get_logger(), msg);
    }

    rclcpp::Subscription<ROSString>::SharedPtr subscription;
};

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ROSTopicSubsriber>());
  rclcpp::shutdown();
  return 0;
}
