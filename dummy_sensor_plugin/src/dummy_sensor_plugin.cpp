#include <gz/sim/System.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gz/plugin/Register.hh>
#include <chrono>
#include <thread>

namespace gz
{
namespace sim
{
namespace systems
{
class DummySensor : public System, public ISystemConfigure
{
public:
    DummySensor()
    {
        // Initialize gz-transport only
        gz_node_ = std::make_shared<gz::transport::Node>();
        publisher_ = gz_node_->Advertise<gz::msgs::StringMsg>("/dummy_sensor");
    }

    void Configure(const Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &/*_sdf*/,
                   EntityComponentManager &_ecm,
                   EventManager &/*_eventMgr*/) override
    {
        // Wait for ROS 2 context to be initialized
        int retries = 10;
        while (!rclcpp::ok() && retries > 0)
        {
            std::cerr << "ROS 2 context not ready, waiting..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Wait 0.5s
            retries--;
        }

        if (!rclcpp::ok())
        {
            std::cerr << "ROS 2 context not initialized after waiting. Skipping ROS 2 functionality." << std::endl;
            // Proceed with gz-transport only
        }
        else
        {
            try
            {
                ros_node_ = std::make_shared<rclcpp::Node>("dummy_sensor_node");
                ros_publisher_ = ros_node_->create_publisher<std_msgs::msg::String>("/dummy_sensor", 10);
            }
            catch (const rclcpp::exceptions::RCLInvalidArgument &e)
            {
                std::cerr << "Failed to initialize ROS 2 node: " << e.what() << std::endl;
                return;  // Skip ROS 2 publishing
            }
        }

        std::string sensor_name = _ecm.Component<components::Name>(_entity)->Data();
        if (ros_node_)
        {
            RCLCPP_INFO(ros_node_->get_logger(), "DummySensor loaded on entity: %s", sensor_name.c_str());
        }
        else
        {
            std::cout << "DummySensor loaded on entity: " << sensor_name << " (ROS 2 unavailable)" << std::endl;
        }

        // Publish via gz-transport (always works)
        gz::msgs::StringMsg gz_msg;
        gz_msg.set_data("Hello World");
        if (publisher_.Publish(gz_msg))
        {
            if (ros_node_) RCLCPP_INFO(ros_node_->get_logger(), "Published 'Hello World' via gz-transport");
            else std::cout << "Published 'Hello World' via gz-transport" << std::endl;
        }
        else
        {
            if (ros_node_) RCLCPP_ERROR(ros_node_->get_logger(), "Failed to publish gz-transport message");
            else std::cerr << "Failed to publish gz-transport message" << std::endl;
        }

        // Publish via ROS 2 (if available)
        if (ros_node_ && ros_publisher_)
        {
            auto ros_msg = std_msgs::msg::String();
            ros_msg.data = "Hello World";
            ros_publisher_->publish(ros_msg);
            RCLCPP_INFO(ros_node_->get_logger(), "Published 'Hello World' to ROS 2 topic /dummy_sensor");
        }
    }

private:
    std::shared_ptr<rclcpp::Node> ros_node_;
    std::shared_ptr<gz::transport::Node> gz_node_;
    gz::transport::Node::Publisher publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ros_publisher_;
};

} // namespace systems
} // namespace sim
} // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::DummySensor,
    gz::sim::System,
    gz::sim::systems::DummySensor::ISystemConfigure
)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::DummySensor, "gz::sim::systems::DummySensor")
