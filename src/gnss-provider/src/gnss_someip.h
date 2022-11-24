#pragma once 

#include <CommonAPI/CommonAPI.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using ROSString = std_msgs::msg::String;


class SomeIpProvider {

public:
    SomeIpProvider() = default;
    ~SomeIpProvider() = default;

    void fireDataEvent(const ROSString & data) {
        RCLCPP_INFO(rclcpp::get_logger("SOMEIP_Provider"), "Sending gnss data over SOME/IP.");
        //RCLCPP_INFO(rclcpp::get_logger("SOMEIP_Provider"), data.data);
    }

};

template <typename T>
class SomeIpReporter : public rclcpp::Node
{
    static constexpr auto node_name = "SOMEIP_Reporter";

    static constexpr auto domain = "local";
    static constexpr auto instance = "SOMEIPServer";
    static constexpr auto timer_duration = 2s;

    static constexpr auto topic = "SOMEIP";
    static constexpr auto qos = 10;

public:
    SomeIpReporter()
        : Node(node_name)
        , someip_provider(std::make_shared<T>())
    {
        if(register_someip_service()) {
            RCLCPP_INFO(this->get_logger(), "SOME/IP has been registered");

            data_subscription = this->create_subscription<ROSString>(topic,
                                                                     qos,
                                                                     std::bind(&SomeIpReporter::on_data,
                                                                               this,
                                                                               std::placeholders::_1));

            publish_timer = this->create_wall_timer(timer_duration, [this]() {            
                RCLCPP_INFO(this->get_logger(), "Timer: Broadcast data over SOME/IP");
        
                std::lock_guard<std::mutex> guard(mutex);

                someip_provider->fireDataEvent(gps_data);
            });
        }
    }

protected:

    bool register_someip_service() {
        if(!CommonAPI::Runtime::get()->registerService(domain,instance, someip_provider)) {
            //TODO: handle error case correctly
            RCLCPP_ERROR(this->get_logger(), "Failed to register SOME/IP");
            return false;
        }

        return true;
    }

    void on_data(const ROSString & msg) 
    {
        std::lock_guard<std::mutex> guard(mutex);

        RCLCPP_INFO(this->get_logger(), "Received raw data from Client node");

        gps_data = msg;
    }

private:
    rclcpp::TimerBase::SharedPtr publish_timer;
    std::shared_ptr<T> someip_provider;

    std::mutex mutex;

    ROSString gps_data;

    rclcpp::Subscription<ROSString>::SharedPtr data_subscription;
};
