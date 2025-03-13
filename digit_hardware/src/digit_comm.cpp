#include "digit_hardware/digit_comm.hpp"

DigitCommunicationNode::DigitCommunicationNode()
    : Node("digit_communication"), command{}, observation{}
{
    this->declare_parameter<bool>("run_simulation", true);
    run_simulation = this->get_parameter("run_simulation").as_bool();
    llapi_init(run_simulation ? simulation_addr : hardware_addr);
    RCLCPP_INFO(this->get_logger(), "Digit Communication Node Initialized");
}

void DigitCommunicationNode::WaitUntilFirstPacket()
{
    while (!llapi_get_observation(&observation))
    {
        switch (run_simulation)
        {
        case true:
            RCLCPP_INFO(this->get_logger(), "Waiting for connection with simulator...");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Waiting for connection with hardware...");
            break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(1));
    }
    command.apply_command = false;
    llapi_send_command(&command);
    llapi_get_limits();
    switch (run_simulation)
        {
        case true:
            RCLCPP_INFO(this->get_logger(), "Connected with simulator");
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Connected with hardware");
            break;
        }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto digit_comm_node = std::make_shared<DigitCommunicationNode>();

    digit_comm_node->WaitUntilFirstPacket();

    rclcpp::spin(digit_comm_node);
    rclcpp::shutdown();
    return 0;
}