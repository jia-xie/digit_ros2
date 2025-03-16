#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import websocket
from std_srvs.srv import Trigger  # ROS 2 Trigger service type

# Agility Robotics WebSocket Server
AGILITY_ADDRESS = "ws://localhost:8080/"  # Change if needed

class AgilityRobotService(Node):
    def __init__(self):
        super().__init__('agility_robot_service')
        self.get_logger().info("Agility Robot Service Node is running...")

        # Create a ROS 2 service to request privilege
        self.srv = self.create_service(Trigger, 'request_privilege', self.handle_request_privilege)

    def request_privilege(self):
        """Continuously requests privilege and mode change until success."""
        try:
            # Connect to WebSocket server
            ws = websocket.create_connection(AGILITY_ADDRESS, subprotocols=['json-v1-agility'])
            self.get_logger().debug("Connected to Agility robot.")

            while rclpy.ok():
                # Send privilege request
                privilege_msg = ["request-privilege", {"privilege": "change-action-command", "priority": 0}]
                ws.send(json.dumps(privilege_msg))
                # self.get_logger().info("Sent privilege request.")

                # Receive response
                response = ws.recv()
                self.get_logger().debug(f"Privilege Response: {response}")

                # Parse JSON response
                try:
                    response_data = json.loads(response)
                    if isinstance(response_data, list) and len(response_data) > 1:
                        if response_data[0] == "privileges" and "privileges" in response_data[1]:
                            for privilege in response_data[1]["privileges"]:
                                if privilege["privilege"] == "change-action-command" and privilege["has"]:
                                    self.get_logger().debug("Privilege granted!")

                                    # Step 2: Send mode change request
                                    if self.set_low_level_api(ws):
                                        ws.close()
                                        self.get_logger().info("Agility robot low level control mode set!")
                                        return True, "Agility robot low level control mode set!"
                                    else:
                                        break  # Retry privilege request if mode change fails

                except json.JSONDecodeError:
                    self.get_logger().error("Failed to parse JSON response for privilege request.")

                self.get_logger().debug("Retrying privilege request...")


        except Exception as e:
            self.get_logger().error(f"Error connecting to Agility API: {e}")
            return False, str(e)

    def set_low_level_api(self, ws):
        """Requests transition to low-level API mode and waits for confirmation."""
        self.get_logger().debug("Requesting transition to low-level API mode...")
        mode_msg = ["action-set-operation-mode", {"mode": "low-level-api"}, 1]
        ws.send(json.dumps(mode_msg))

        while rclpy.ok():
            # Wait for response
            response = ws.recv()
            self.get_logger().debug(f"Mode Change Response: {response}")

            # Parse response
            try:
                mode_data = json.loads(response)
                if isinstance(mode_data, list) and len(mode_data) > 1:
                    if mode_data[0] == "action-status-changed" and "status" in mode_data[1]:
                        status = mode_data[1]["status"]
                        if status == "success":
                            self.get_logger().debug("Successfully transitioned to low-level API mode.")
                            return True
                        elif status == "failure":
                            error_info = mode_data[1].get("info", "Unknown error")
                            self.get_logger().debug(f"Mode transition failed: {error_info}. Retrying...")
                            return False
            except json.JSONDecodeError:
                self.get_logger().error("Failed to parse JSON response for mode transition.")


    def handle_request_privilege(self, request, response):
        """Handles the incoming service request."""
        self.get_logger().debug("Received privilege request...")
        
        # Keep trying until success
        success, message = self.request_privilege()
        
        # Populate ROS 2 response
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
