import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode, CommandBool
import time

class MissionStarter(Node):
    def __init__(self):
        super().__init__('mission_starter')

        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')

        self.get_logger().info("Waiting for services...")
        self.set_mode_client.wait_for_service()
        self.arm_client.wait_for_service()

        self.start_mission_sequence()

    def set_mode(self, mode_name):
        req = SetMode.Request()
        req.custom_mode = mode_name
        self.get_logger().info(f"Setting mode: {mode_name}")
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().mode_sent:
            self.get_logger().info(f"Mode set to {mode_name}")
            return True
        else:
            self.get_logger().error(f"Failed to set mode: {mode_name}")
            return False

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        self.get_logger().info("Arming drone...")
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info("Drone armed successfully.")
            return True
        else:
            self.get_logger().error("Failed to arm drone.")
            return False

    def start_mission_sequence(self):
        if self.set_mode("GUIDED"):
            time.sleep(1)
            if self.arm():
                time.sleep(1)
                self.set_mode("AUTO")

def main(args=None):
    rclpy.init(args=args)
    node = MissionStarter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
