import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist  # Para comandos de velocidade
from cv_bridge import CvBridge
import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('teste_controle.py')
        self.bridge = CvBridge()
        
        # Subscriber para /camera
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        # Publisher para imagem processada
        self.publisher = self.create_publisher(Image, '/processed_image', 10)
        
        # Novo: Publisher para comandos de velocidade
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('Nó de visão iniciado. Subscrevendo /camera')

    def image_callback(self, msg):
        try:
            # Converter ROS para OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Processamento (use Canny + Hough ou threshold)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            blurred_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
            edges = cv2.Canny(blurred_image, 50, 150)
            
            # Detectar posição da linha (usando momentos ou Hough)
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
            
            # Calcular centro da linha detectada
            image_center = edges.shape[1] // 2  # Centro horizontal da imagem
            line_center = image_center  # Padrão: centro
            if lines is not None and len(lines) > 0:
                # Usar a linha mais longa ou média
                longest_line = max(lines, key=lambda l: np.sqrt((l[0][2]-l[0][0])**2 + (l[0][3]-l[0][1])**2))
                x1, y1, x2, y2 = longest_line[0]
                line_center = (x1 + x2) // 2  # Centro horizontal da linha
            
            # Calcular erro (offset da linha em relação ao centro)
            error = line_center - image_center
            
            # Controle simples: Velocidade linear constante, angular proporcional ao erro
            twist = Twist()
            twist.linear.x = 0.2  # Velocidade para frente (ajuste para velocidade do robô)
            twist.angular.z = -error * 0.005  # Ganho proporcional (ajuste: negativo para correção)
            # Ex.: Se linha à direita (error > 0), gira para esquerda (angular.z negativo)
            
            # Publicar comando de velocidade
            self.cmd_publisher.publish(twist)
            
            # Desenhar linhas e publicar imagem (opcional)
            image_with_lines = cv_image.copy()
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(image_with_lines, (x1, y1), (x2, y2), (0, 0, 255), 2)
            processed_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
            self.publisher.publish(processed_msg)
            
            self.get_logger().info(f'Erro: {error}, Velocidade: linear={twist.linear.x}, angular={twist.angular.z}')
            
        except Exception as e:
            self.get_logger().error(f'Erro: {str(e)}')

# main permanece igual
