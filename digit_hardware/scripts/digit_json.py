#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import asyncio
import agility
import agility.messages as msgs
from std_srvs.srv import Trigger  # Import the Trigger service type

class AgilityRobotService(Node):
    def __init__(self):
        super().__init__('agility_robot_service')
        self.get_logger().info("Agility Robot Service Node is running...")

        # Create a ROS 2 service to request privilege
        self.srv = self.create_service(Trigger, 'request_privilege', self.handle_request_privilege)

    async def request_privilege(self):
        """Requests privilege from the Agility Robotics API asynchronously."""
        try:
            async with agility.JsonApi() as api:
                await api.request_privilege('change-action-command')
                self.get_logger().info("Privilege granted!")
                return True, "Agility robot privilege acquired!"
        except Exception as e:
            self.get_logger().error(f"Error connecting to Agility API: {e}")
            return False, str(e)

    def handle_request_privilege(self, request, response):
        """Handles the incoming service request."""
        self.get_logger().info("Received privilege request...")
        success, message = asyncio.run(self.request_privilege())  # Run async function
        response.success = success
        response.message = message
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AgilityRobotService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
