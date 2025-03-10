import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        
        # Initialize the OpenCV capture
        self.cap = cv2.VideoCapture(0)  # Use 0 for the default webcam
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open webcam")
            return
        

        # Create a QoS Profile to prevent re-sending failed image messages, that can cause network issues with such data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        
        # Create a publisher for the camera feed
        self.publisher = self.create_publisher(Image, 'camera_raw', qos_profile)
        
        # Create a CvBridge instance
        self.bridge = CvBridge()
        
        # Start a timer to publish frames every 0.1 seconds
        self.timer = self.create_timer(0.1, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to capture image')
            return
        
        # Convert the frame to a ROS Image message
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        
        # Publish the message
        self.publisher.publish(msg)

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
