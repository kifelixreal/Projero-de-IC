import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        
        # Subscriber ajustado para /camera
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        # Publisher para imagem processada
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        
        self.get_logger().info('Nó de visão iniciado. Subscrevendo /camera')

    def image_callback(self, msg):
        try:
            # Converter ROS Image para OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 1. Cinza
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 2. Blur para ruído
            blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
            
            # 3. Canny Edge Detection
            edges = cv2.Canny(blurred_image, 50, 150)
            
            # 4. Hough Line Transform
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 70, minLineLength=50, maxLineGap=10)
            
            # 5. Desenhar linhas detectadas na imagem original
            image_with_lines = cv_image.copy()
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(image_with_lines, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            # 6. Publicar imagem com linhas
            #processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8') #saida do canny
            processed_msg = self.bridge.cv2_to_imgmsg(image_with_lines, encoding='bgr8') #colorido
            #processed_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(image_with_lines, cv2.COLOR_BGR2GRAY), encoding='mono8')  # Converte para cinza o contorno
            self.publisher.publish(processed_msg)
            
            # Log detalhado
            num_lines = len(lines) if lines is not None else 0
            self.get_logger().info(f'Processado. Linhas detectadas: {num_lines}')
            
            # (Debug) Salvar frame para teste offline
            # cv2.imwrite('debug_frame.jpg', cv_image)
            # cv2.imwrite('debug_edges.jpg', edges)
            # cv2.imwrite('debug_lines.jpg', image_with_lines)
            
        except Exception as e:
            self.get_logger().error(f'Erro: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

