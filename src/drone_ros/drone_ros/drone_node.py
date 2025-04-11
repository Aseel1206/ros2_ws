import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time

class ArmTakeoffNode(Node):
    def __init__(self):
        super().__init__('arm_takeoff_node')

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.wait_for_services()
        self.arm_and_takeoff()

    def wait_for_services(self):
        self.get_logger().info('Waiting for MAVROS services...')
        for client in [self.arm_client, self.mode_client, self.takeoff_client]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {client.srv_name}...')

    def arm_and_takeoff(self, target_alt=10.0):
        # Set mode to GUIDED
        mode_req = SetMode.Request()
        mode_req.custom_mode = 'GUIDED'
        mode_future = self.mode_client.call_async(mode_req)
        rclpy.spin_until_future_complete(self, mode_future)
        self.get_logger().info(f"Set mode result: {mode_future.result().mode_sent}")

        time.sleep(2)

        # Arm the drone
        arm_req = CommandBool.Request()
        arm_req.value = True
        arm_future = self.arm_client.call_async(arm_req)
        rclpy.spin_until_future_complete(self, arm_future)
        self.get_logger().info(f"Arm result: {arm_future.result().success}")

        time.sleep(2)

        # Takeoff
        takeoff_req = CommandTOL.Request()
        takeoff_req.altitude = target_alt
        takeoff_req.latitude = 0.0
        takeoff_req.longitude = 0.0
        takeoff_req.min_pitch = 0.0
        takeoff_req.yaw = 0.0
        takeoff_future = self.takeoff_client.call_async(takeoff_req)
        rclpy.spin_until_future_complete(self, takeoff_future)
        self.get_logger().info(f"Takeoff result: {takeoff_future.result().success}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmTakeoffNode()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
