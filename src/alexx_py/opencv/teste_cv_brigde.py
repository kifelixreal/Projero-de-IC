import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        
        # Subscriber ajustado para /camera (conforme seu launch)
        self.subscription = self.create_subscription(
            Image,
            '/camera',  # Tópico exato do launch
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
            
            # 3. Thresholding adaptativo (captura linha inteira, mesmo com iluminação variável)
            binary_image = cv2.adaptiveThreshold(
                blurred_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
            )
            
            # 4. Morfologia: Dilatação para conectar partes da linha
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            binary_image = cv2.dilate(binary_image, kernel, iterations=1)  # Aumente para 2 se necessário
            
            # 5. Contornos
            contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # 6. Filtrar contornos por área (focar na linha, ignorar ruídos)
            min_area = 500  # Ajuste: Teste com frames salvos (ex.: 1000+)
            filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]
            
            # 7. Desenhar contornos na original
            image_with_contours = cv_image.copy()
            cv2.drawContours(image_with_contours, filtered_contours, -1, (0, 0, 255), 2)
            
            # Publicar binarizada
            processed_msg = self.bridge.cv2_to_imgmsg(binary_image, encoding='mono8')
            self.publisher.publish(processed_msg)
            
            # Log detalhado
            self.get_logger().info(f'Processado. Contornos válidos: {len(filtered_contours)}')
            
            # (Debug) Salvar frame para teste offline (descomente temporariamente)
            # cv2.imwrite('debug_frame.jpg', cv_image)
            # cv2.imwrite('debug_binary.jpg', binary_image)
            
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