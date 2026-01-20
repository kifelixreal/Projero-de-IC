import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes

from sensor_msgs.msg import Image  # ROS message type for images
from cv_bridge import CvBridge  # Converts ROS Image <-> OpenCV image (numpy array)

from geometry_msgs.msg import PointStamped, Vector3Stamped  # Geometry messages for centroid + error
import cv2  # OpenCV
import numpy as np  # Numerical operations


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')  # Initialize the node with a name

        # CvBridge instance for conversions between ROS images and OpenCV images
        self.bridge = CvBridge()

        # ---- Parameters (adjust these) ----
        # Percentage of the image height used as ROI at the bottom (e.g., 0.65 means bottom 35%)
        self.baixo_linhas_ratio = 0.65

        # Canny thresholds (adjust based on your lighting/contrast)
        self.canny_low = 50
        self.canny_high = 150

        # Minimum number of edge pixels in ROI to consider the line "found"
        self.min_edge_points = 80
        

        # ---- Subscribers ----
        # Subscribe to the camera topic
        self.sub_image = self.create_subscription(
            Image,                 # Message type
            '/camera',            # Topic name
            self.image_callback,  # Callback function
            10                    # Queue depth
        )

        # ---- Publishers ----
        # Publish centroid as a PointStamped (x,y in pixels; z used as "confidence")
        self.pub_centroid = self.create_publisher(PointStamped, '/line_centroid', 10)

        # Publish error as Vector3Stamped:
        #   x = normalized error (~[-1,1])
        #   y = error in pixels
        #   z = number of edge points (confidence)
        self.pub_error = self.create_publisher(Vector3Stamped, '/line_error', 10)

        # Optional: publish a debug image showing Canny edges (mono8)
        self.pub_debug = self.create_publisher(Image, '/processed_image', 10)

        # Log node start
        self.get_logger().info('VisionNode started: /camera -> /line_centroid, /line_error, /processed_image')

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image to OpenCV BGR image (uint8)
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale (Canny works on single channel)
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

            # Blur to reduce noise and stabilize edges
            gray = cv2.GaussianBlur(gray, (5, 5), 0)

            # Canny edge detection (output is a binary-like image: edges are 255)
            edges = cv2.Canny(gray, self.canny_low, self.canny_high)

            # Image size
            h, w = edges.shape

            # Define ROI (bottom part of image)
            y0 = int(h * self.baixo_linhas_ratio)  # start row of ROI
            roi = edges[y0:h, :]  # ROI edges

            # Find coordinates of edge pixels in ROI (where pixel > 0)
            ys, xs = np.where(roi > 0)  # ys and xs are arrays of indices inside ROI

            # Count edge points (used for confidence and "line found")
            n_points = int(xs.size)

            # Prepare centroid message (always publish something)
            centroid_msg = PointStamped()
            centroid_msg.header = msg.header  # reuse timestamp/frame_id from camera message

            # Prepare error message (always publish something)
            error_msg = Vector3Stamped()
            error_msg.header = msg.header

            # If not enough edge points, consider line not found
            if n_points < self.min_edge_points:
                # Centroid undefined: publish center as fallback
                centroid_msg.point.x = float(w / 2.0)
                centroid_msg.point.y = float((y0 + h) / 2.0)
                centroid_msg.point.z = float(n_points)  # confidence

                # Error = 0, confidence = n_points
                error_msg.vector.x = 0.0  # normalized error
                error_msg.vector.y = 0.0  # pixel error
                error_msg.vector.z = float(n_points)

                # Publish messages
                self.pub_centroid.publish(centroid_msg)
                self.pub_error.publish(error_msg)

                # Publish debug image (optional)
                debug_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
                self.pub_debug.publish(debug_msg)

                return

            # Compute centroid in ROI coordinates (mean of edge pixel positions)
            cx_roi = float(np.mean(xs))  # x centroid inside ROI
            cy_roi = float(np.mean(ys))  # y centroid inside ROI

            # Convert ROI centroid to full-image coordinates
            cx = cx_roi
            cy = cy_roi + float(y0)

            # Compute error relative to image center (in pixels)
            center_x = float(w / 2.0)
            error_px = cx - center_x

            # Normalize error to roughly [-1, 1] by dividing by half-width
            error_norm = error_px / center_x

            # Fill centroid message
            centroid_msg.point.x = cx
            centroid_msg.point.y = cy
            centroid_msg.point.z = float(n_points)  # store confidence in z

            # Fill error message
            error_msg.vector.x = float(error_norm)
            error_msg.vector.y = float(error_px)
            error_msg.vector.z = float(n_points)

            # Publish centroid + error
            self.pub_centroid.publish(centroid_msg)
            self.pub_error.publish(error_msg)

            # Publish debug image (edges) so you can visualize in image_view
            debug_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
            self.pub_debug.publish(debug_msg)

        except Exception as e:
            # Log any error to help debugging
            self.get_logger().error(f'VisionNode error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)      # Initialize ROS
    node = VisionNode()        # Create node instance
    rclpy.spin(node)           # Keep node alive processing callbacks
    node.destroy_node()        # Cleanup
    rclpy.shutdown()           # Shutdown ROS


if __name__ == '__main__':
    main()
