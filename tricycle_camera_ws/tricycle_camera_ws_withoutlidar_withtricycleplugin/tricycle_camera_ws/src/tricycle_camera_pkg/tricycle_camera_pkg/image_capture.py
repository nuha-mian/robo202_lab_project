#The shebang line 
#!/usr/bin/env python3
# import libraries and interfaces
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# create a new class that inherits from the rclpy Node class
class ImageCapture(Node):
    def __init__(self):
        # In this class, call the parent constructor with super()
        super().__init__('image_capture')
        self.image_value = None
        self.image_count = 0
        
        # subscribe to camera topic
        self.image_sub = self.create_subscription(Image, '/camera1/image_raw', self.image_callback, 10)
        
        # publisher to re-publish camera images
        self.image_pub = self.create_publisher(Image, '/captured_image', 10)
        
        self.get_logger().info('Image Capture Node Started')
        self.get_logger().info('Subscribed to: /camera1/image_raw')
        self.get_logger().info('Publishing to: /captured_image')
    
    def image_callback(self, msg):
        # store incoming image data
        self.image_value = msg
        self.image_count += 1
        
        # re-publish the image continuously
        self.image_pub.publish(msg)
        
        # Log basic information
        self.get_logger().info(f'Image #{self.image_count} published - Size: {msg.width}x{msg.height}, Encoding: {msg.encoding}')

# Create a main() function 
def main(args=None):
    rclpy.init(args=args)  # initialise ROS2 communication  
    node = ImageCapture()  # create an object of the class 
    rclpy.spin(node)  # start Node execution
    rclpy.shutdown()  # shutdown ROS2 communication

# call the main() function 
if __name__ == '__main__':
    main()
