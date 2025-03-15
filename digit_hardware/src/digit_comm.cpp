#include "digit_hardware/digit_comm.hpp"

DigitCommunicationNode::DigitCommunicationNode()
    : Node("digit_communication"), command{}, observation{}
{
    this->declare_parameter<bool>("run_simulation", true);
    run_simulation = this->get_parameter("run_simulation").as_bool();
    privilege_client_ = this->create_client<std_srvs::srv::Trigger>("request_privilege");

    
    llapi_init(run_simulation ? simulation_addr : hardware_addr);
    
    RCLCPP_INFO(this->get_logger(), "Digit Communication Node Initialized");
}

void DigitCommunicationNode::RequestPrivilege()
{
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    while (!privilege_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "privilege server is busy, waiting again...");
    }
    auto result = privilege_client_->async_send_request(request);
}

void DigitCommunicationNode::WaitUntilConnected()
{
    while (!llapi_get_observation(&observation) && rclcpp::ok())
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
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
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
    // RequestPrivilege();
}

void DigitCommunicationNode::Test()
{
    // if (!llapi_connected()) {
    //     RCLCPP_ERROR(this->get_logger(), "Low Level Connection lost");
    // }
    command.fallback_opmode = Damping;
    command.apply_command = true;
    llapi_send_command(&command);
    // if (llapi_get_observation(&observation))
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received Observation: %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
    //                 observation.motor.position[0], observation.motor.position[1], observation.motor.position[2],
    //                 observation.motor.position[3], observation.motor.position[4], observation.motor.position[5],
    //                 observation.motor.position[6], observation.motor.position[7], observation.motor.position[8],
    //                 observation.motor.position[9], observation.motor.position[10], observation.motor.position[11],
    //                 observation.motor.position[12], observation.motor.position[13], observation.motor.position[14]);
    // }
    // else
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Failed to get observation");
    // }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto digit_comm_node = std::make_shared<DigitCommunicationNode>();

    digit_comm_node->WaitUntilConnected();
    while (rclcpp::ok())
    {
        // digit_comm_node->Test();
        rclcpp::sleep_for(std::chrono::milliseconds(100));

    }
    rclcpp::spin(digit_comm_node);
    rclcpp::shutdown();
    return 0;
}