import time
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush, SetMode, CommandBool, CommandLong , WaypointSetCurrent ,  CommandTOL
from mavros_msgs.msg import Waypoint

class MissionUploaderFromFile(Node):
    def __init__(self):
        super().__init__('mission_uploader_from_file')

        self.waypoint_file = '/home/aseel/SUAS_24.waypoints'
        self.get_logger().info("Reading mission from file...")

        self.declare_parameter('mission_file', self.waypoint_file)
        self.waypoint_file = self.get_parameter('mission_file').value

        self.load_and_upload_mission()

    def load_and_upload_mission(self):
        waypoints = self.parse_waypoint_file(self.waypoint_file)
        if not waypoints:
            self.get_logger().error("No waypoints found.")
            return

        self.push_mission(waypoints)

        self.set_mode("GUIDED")
        time.sleep(1)
        self.arm(True)
        time.sleep(1)
        
        #self.set_mode("AUTO")
        #time.sleep(1)
        self.start_mission()
       

    def parse_waypoint_file(self, file_path):
        waypoints = []
        with open(file_path, 'r') as file:
            lines = file.readlines()

        if not lines or not lines[0].startswith("QGC WPL"):
            self.get_logger().error("Invalid or empty waypoint file.")
            return []

        for line in lines[1:]:
            parts = line.strip().split('\t')
            if len(parts) < 12:
                continue
            wp = Waypoint()
            wp.is_current = bool(int(parts[1]))
            wp.frame = int(parts[2])
            wp.command = int(parts[3])
            wp.param1 = float(parts[4])
            wp.param2 = float(parts[5])
            wp.param3 = float(parts[6])
            wp.param4 = float(parts[7])
            wp.x_lat = float(parts[8])
            wp.y_long = float(parts[9])
            wp.z_alt = float(parts[10])
            wp.autocontinue = bool(int(parts[11]))
            waypoints.append(wp)

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints from file.")
        return waypoints

    def push_mission(self, waypoints):
        client = self.create_client(WaypointPush, '/mavros/mission/push')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/mission/push service...')
        req = WaypointPush.Request()
        req.start_index = 0
        req.waypoints = waypoints
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f"Pushed {future.result().wp_transfered} waypoints successfully.")
        else:
            self.get_logger().error("Failed to push waypoints.")

    def set_mode(self, mode_str):
        client = self.create_client(SetMode, '/mavros/set_mode')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        req = SetMode.Request()
        req.custom_mode = mode_str
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Set mode to {mode_str}")

    def arm(self, value=True):
        client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/arming service...')
        req = CommandBool.Request()
        req.value = value
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info("Vehicle armed.")
        else:
            self.get_logger().error("Failed to arm.")

    

    def start_mission(self):
        # Takeoff before mission starts
        self.takeoff(altitude=10.0)

        # Set mode to AUTO.MISSION
        self.set_mode("AUTO")

        # (Optional) Set the current waypoint to 0
        client = self.create_client(WaypointSetCurrent, '/mavros/mission/set_current')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/mission/set_current service...')

        req = WaypointSetCurrent.Request()
        req.wp_seq = 0
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("✅ Mission started from waypoint 0.")
        else:
            self.get_logger().error("❌ Failed to start mission.")

    def takeoff(self, altitude=10.0):
        client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/cmd/takeoff service...')

        req = CommandTOL.Request()
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0  # Use 0 if GPS not needed
        req.longitude = 0.0
        req.altitude = altitude

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f"✅ Takeoff command sent, altitude: {altitude}m")
        else:
            self.get_logger().error("❌ Takeoff failed.")

def main(args=None):
    rclpy.init(args=args)
    node = MissionUploaderFromFile()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
