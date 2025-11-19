#The shebang line 
#!/usr/bin/env python3
# import libraries and interfaces
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
import numpy as np
import math

# create a new class that inherits from the rclpy Node class
class ImageCapture(Node):
    def __init__(self):
        # In this class, call the parent constructor with super()
        super().__init__('image_capture')
        self.image_value = None
        self.scan_value = None
        
        # subscribe to camera topic
        self.image_sub = self.create_subscription(Image, '/camera1/image_raw', self.image_callback, 10)
        # subscribe to lidar/scan topic (to measure distance like ObsAvoid)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.laserscan_callback, 10)

        # publisher to announce image capture status
        self.status_pub = self.create_publisher(String, '/image_capture_status', 10)
        # publisher to re-publish camera images
        self.image_pub = self.create_publisher(Image, '/captured_image', 10)

        # check periodically
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def image_callback(self, msg):
        # store incoming image data
        self.image_value = msg
        # re-publish the image continuously
        self.image_pub.publish(msg)

    def laserscan_callback(self, msg):
        # store latest lidar/scan message
        self.scan_value = msg
    
    def timer_callback(self):
        # Use lidar/scan to determine distance (same logic as ObsAvoid)
        if self.scan_value:
            ranges = list(self.scan_value.ranges)
            n = len(ranges)
            if n == 0:
                return
            center = n // 2
            left = min(n - 1, center + 45)
            right = max(0, center - 45)

            sector = ranges[right:left + 1]
            # filter out invalid readings
            valid = [r for r in sector if r is not None and not math.isinf(r) and not math.isnan(r) and r > 0.0]

            # If any valid range is less than or equal to 0.5 m then we "capture" the image
            status_msg = String()
            if any(r <= 0.5 for r in valid):
                status_msg.data = 'Image captured'
                self.status_pub.publish(status_msg)
                self.get_logger().info('Image captured')
            else:
                status_msg.data = 'Image not captured. Move closer'
                self.status_pub.publish(status_msg)
                self.get_logger().info('Image not captured. Move closer')
        else:
            # no scan message yet; optionally wait
            self.get_logger().debug('Waiting for /scan messages to determine distance')

# Create a main() function 
def main(args=None):
    rclpy.init(args=args)  # initialise ROS2 communication  
    node = ImageCapture()  # create an object of the class 
    rclpy.spin(node)  # start Node execution
    rclpy.shutdown()  # shutdown ROS2 communication

# call the main() function 
if __name__ == '__main__':
    main()
