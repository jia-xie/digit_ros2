#ifndef DIGIT_COMMUNICATION_NODE_HPP
#define DIGIT_COMMUNICATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

extern "C" {
#include "lowlevelapi.h"
}

class DigitCommunicationNode : public rclcpp::Node
{
public:
    DigitCommunicationNode();
    ~DigitCommunicationNode()
    {
        llapi_free();
    }

    void WaitUntilConnected();
    void Test();

private:
    llapi_command_t command;
    llapi_observation_t observation;
    const char *simulation_addr = "127.0.0.1";
    const char *hardware_addr = "10.10.1.1";
    bool run_simulation;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr privilege_client_;

    void RequestPrivilege();
};

#endif // DIGIT_COMMUNICATION_NODE_HPP
