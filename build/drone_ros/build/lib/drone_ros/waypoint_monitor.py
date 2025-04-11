import rclpy
from rclpy.node import Node
from mavros_msgs.msg import WaypointReached
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy
import piexif
from PIL import Image
import shutil
import os
from threading import Timer
from datetime import datetime
import subprocess

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

class WaypointMonitor(Node):
    def __init__(self):
        super().__init__('waypoint_monitor')

        self.create_subscription(WaypointReached, '/mavros/mission/reached', self.reached_callback, 10)
        self.create_subscription(Imu, '/mavros/imu/data', self.imu_callback,  qos_profile)
        self.create_subscription(NavSatFix, '/mavros/global_position/global', self.gps_callback, qos_profile)
        self.create_subscription(Float64, '/mavros/global_position/rel_alt', self.alt_callback, qos_profile)
        self.create_subscription(Float64, '/mavros/global_position/compass_hdg', self.heading_callback, qos_profile)

        self.image_source_path = '/home/aseel/demo.png'
        self.captured_folder = '/home/aseel/captured'
        os.makedirs(self.captured_folder, exist_ok=True)

        self.saving_images = False
        self.image_copy_timer = None

        self.gps_timestamp = None
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.heading = None

        self.get_logger().info("Waypoint Monitor Node started. Waiting for waypoints...")

    def imu_callback(self, msg):
        pass  # Optional: process IMU data if needed

    def gps_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        stamp = msg.header.stamp
        self.gps_timestamp = stamp.sec + stamp.nanosec * 1e-9

    def alt_callback(self, msg):
        self.altitude = msg.data

    def heading_callback(self, msg):
        self.heading = msg.data

    def reached_callback(self, msg):
        self.get_logger().info(f"Waypoint {msg.wp_seq} reached.")

        if msg.wp_seq == 15:
            self.get_logger().info("Start capturing images.")
            self.saving_images = True
            self.start_image_copy_loop()

        elif msg.wp_seq == 20:
            self.get_logger().info("Stop capturing images.")
            self.saving_images = False
            if self.image_copy_timer:
                self.image_copy_timer.cancel()

    def start_image_copy_loop(self):
        if not self.saving_images:
            return

        self.copy_image()
        self.image_copy_timer = Timer(1.0, self.start_image_copy_loop)
        self.image_copy_timer.start()

    def copy_image(self):
        if not os.path.exists(self.image_source_path):
            self.get_logger().warn(f"Image source not found: {self.image_source_path}")
            return

        if self.gps_timestamp is None:
            self.get_logger().warn("No GPS timestamp available yet.")
            return

        dt = datetime.fromtimestamp(self.gps_timestamp)
        timestamp_str = dt.strftime('%Y%m%d_%H%M%S_%f')
        dest_path = os.path.join(self.captured_folder, f'image_{timestamp_str}.jpg')

        self.get_logger().info(f'Latitude:{self.latitude} Longitude:{self.longitude} Altitude:{self.altitude}')

        try:
            
            with Image.open(self.image_source_path) as img:
                rgb_img = img.convert('RGB')  
                rgb_img.save(dest_path, "JPEG")
           
           
            if all(v is not None for v in [self.latitude, self.longitude, self.altitude, self.heading]):
                def to_deg(value):
                    deg = int(value)
                    min_ = int((value - deg) * 60)
                    sec = int(((value - deg - min_ / 60) * 3600) * 100)
                    return ((deg, 1), (min_, 1), (sec, 100))

                gps_ifd = {
                    piexif.GPSIFD.GPSLatitudeRef: b'N' if self.latitude >= 0 else b'S',
                    piexif.GPSIFD.GPSLatitude: to_deg(abs(self.latitude)),
                    piexif.GPSIFD.GPSLongitudeRef: b'E' if self.longitude >= 0 else b'W',
                    piexif.GPSIFD.GPSLongitude: to_deg(abs(self.longitude)),
                    piexif.GPSIFD.GPSAltitudeRef: 0,
                    piexif.GPSIFD.GPSAltitude: (int(self.altitude * 100), 100),
                    piexif.GPSIFD.GPSImgDirectionRef: b'T',
                    piexif.GPSIFD.GPSImgDirection: (int(self.heading * 100), 100),
                }

                exif_dict = {"GPS": gps_ifd}
                exif_bytes = piexif.dump(exif_dict)

                piexif.insert(exif_bytes, dest_path)

                self.get_logger().info(f"Metadata added to {dest_path}")
            else:
                self.get_logger().warn("Incomplete GPS/heading data. Metadata not added.")

            self.get_logger().info(f"Image saved: {dest_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to copy image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = WaypointMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



         