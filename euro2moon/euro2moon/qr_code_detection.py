import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import zxingcpp
import numpy as np

class PublisherSubscriberNode(Node):
    def __init__(self):
        super().__init__('qr_code_detection')

        #C onverts ROS image to OpenCV image
        self.bridge = CvBridge() 

        # Latest frame storage
        self.latest_frame = None


        # Create a QoS Profile to prevent re-sending failed image messages, that can cause network issues with such data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # Publish live feed to 'detection_feed'
        self.detectionFeedPublisher = self.create_publisher(Image, 'detection_feed', qos_profile)

        # Publish still to 'detection'
        self.publisher_2 = self.create_publisher(Image,'detection', 10)



        # Subscribe to camera_raw
        self.create_subscription(Image,'camera_raw', self.camera_callback, qos_profile)
       
        # Subscribe to take_picture
        self.create_subscription(String,'take_picture', self.listener_callback, 10) # What datatype will take_picture publish? Assumed string



    def detect_qr(self):
        if self.latest_frame is None:
            self.get_logger().warning("No frame available for QR detection")
            return
        
        # Convert ROS Image message to OpenCV format
        frame = self.latest_frame.copy()

        # Detect QR codes in the frame
        decoded_objects = zxingcpp.read_barcodes(frame)

        for obj in decoded_objects:
            # Default color is red for detected QR codes
            color = (0, 0, 255)            

            # If data can be read, set the color to green
            if obj.text and obj.content_type == zxingcpp.BarcodeFormat.QRCode:
                self.get_logger().info(f"QR Code detected: {obj.text}")
                color = (0, 255, 0)
                
            # Draw polygon around the QR code
            pts = np.intp([(obj.position.top_left.x, obj.position.top_left.y), (obj.position.top_right.x, obj.position.top_right.y), (obj.position.bottom_right.x, obj.position.bottom_right.y), (obj.position.bottom_left.x, obj.position.bottom_left.y)])
            cv2.polylines(frame, [pts], isClosed=True, color=color, thickness=2)
  

        # Convert the modified frame back to ROS Image message
        processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.detectionFeedPublisher.publish(processed_msg)



    # When signal received from take_picture, publish still image to detection
    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data} from take_picture. Publishing still image...')
        
        if self.latest_frame is None:
            self.get_logger().warning("No frame available to publish")
            return
        
        # Convert the latest frame to a ROS Image message
        still_image_msg = self.bridge.cv2_to_imgmsg(self.latest_frame, encoding='bgr8')
        
        # Publish to detection
        self.publisher_2.publish(still_image_msg)



    # When signal received from camera, convert image to OpenCV type, check for qr code
    def camera_callback(self,msg):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.detect_qr()




def main(args=None):
    rclpy.init(args=args)
    node = PublisherSubscriberNode()

    rclpy.spin(node)  # Keep the node running

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
