#include <types/conversion.h>

#include "gnss_someip.h"
#include "gpsd_client.h"

auto main(int argc, char **argv) -> int 
{
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    //auto gnss_someip_reporter_node = std::make_shared<SomeIpReporter>();
    auto ROSClientNode = std::make_shared<ROSClient>();

    //executor.add_node(gnss_someip_reporter_node);
    executor.add_node(ROSClientNode);

    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
