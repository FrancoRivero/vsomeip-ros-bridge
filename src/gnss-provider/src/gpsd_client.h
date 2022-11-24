#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>

#include <std_msgs/msg/string.hpp>

using ROSString = std_msgs::msg::String;


// inspired by https://gist.github.com/ncoder-1/8313815ac387e6757f751dc8960f03d7
class ROSClient : public rclcpp::Node {

    static constexpr auto node_name = "ROS_Client_node";

    static constexpr auto gpsd_host = "localhost";
    static constexpr auto waiting_time = 1000000;

    static constexpr auto topic = "ROS";
    static constexpr auto qos = 10;

    static constexpr auto gpsd_read_timer_delay = 1000ms;

public:
    ROSClient() : Node(node_name)
    {
        publisher = this->create_publisher<ROSString>(topic, qos);

        timer = this->create_wall_timer(gpsd_read_timer_delay, std::bind(&ROSClient::read, this));

        //TODO: handle case when there is no connection to gpsd
    }
    ~ROSClient() = default;

    void read() {
        while(true)
        {
            std::lock_guard<std::mutex> lock_guard(mutex);

            RCLCPP_WARN(this->get_logger(), "Obtained data");

            auto message = std_msgs::msg::String();
            message.data = "Hello, world! ";
            
            publisher->publish(message);

            return;
        }
    }

protected:

private:
    std::mutex mutex;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<ROSString>::SharedPtr publisher;
};
