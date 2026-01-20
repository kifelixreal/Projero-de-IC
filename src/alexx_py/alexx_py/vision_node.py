import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from geometry_msgs.msg import PointStamped, Vector3Stamped

import cv2
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()

        # Parâmetros
        self.baixo_linhas_ratio = 0.65  # pega 35% inferior da imagem como ROI

        # Faixa HSV para branco (ajuste conforme o cenário)
        self.lower_white = np.array([0, 0, 200])    # H=0-180, S=0-30 (saturação baixa), V alto
        self.upper_white = np.array([180, 30, 255])

        self.min_edge_points = 80  # mínimo de pixels para considerar linha encontrada

        # Filtro suavizador pro erro (EMA)
        self.alpha = 0.7
        self.error_filtered = 0.0

        # Kernel para morfologia
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

        # Subscreve /camera
        self.sub = self.create_subscription(Image, '/camera', self.image_callback, 10)

        # Publica ponto centróide da linha (x,y) e confiança no z
        self.pub_centroid = self.create_publisher(PointStamped, '/line_centroid', 10)

        # Publica erro e confiança para o controle
        self.pub_error = self.create_publisher(Vector3Stamped, '/line_error', 10)

        # Publica uma imagem binária para debug
        self.pub_debug = self.create_publisher(Image, '/processed_image', 10)

        self.get_logger().info('VisionNode iniciado (segmentação linha branca em HSV).')

    def image_callback(self, msg: Image):
        try:
            # Passa ROS Image para OpenCV BGR
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            h, w, _ = bgr.shape

            # Converte para HSV (faixa de cor mais fácil para segmentação)
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            # Máscara selecionando pixels “brancos” segundo a faixa HSV
            mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)

            # Aplica morfologia para limpar a máscara (remover ruídos, fechar pequenos furos)
            mask_clean = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, self.kernel)
            mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, self.kernel)

            # Define ROI no final da imagem para análise da linha
            y0 = int(h * self.baixo_linhas_ratio)
            roi = mask_clean[y0:h, :]

            # Conta pixels brancos na ROI (confiança)
            n_points = int(np.count_nonzero(roi))

            # Inicializa mensagens para publicar
            centroid_msg = PointStamped()
            centroid_msg.header = msg.header

            error_msg = Vector3Stamped()
            error_msg.header = msg.header

            # Caso não tenha pontos suficientes, considera linha perdida
            if n_points < self.min_edge_points:
                # Centróide no meio (fallback)
                cx = w / 2.0
                cy = (y0 + h) / 2.0
                error_px = 0.0
                error_norm = 0.0

                # Reseta filtro para erro
                self.error_filtered = 0.0

                # Prepara mensagens publicadas
                centroid_msg.point.x = float(cx)
                centroid_msg.point.y = float(cy)
                centroid_msg.point.z = float(n_points)

                error_msg.vector.x = float(error_norm)
                error_msg.vector.y = float(error_px)
                error_msg.vector.z = float(n_points)

                # Publica e retorna
                self.pub_centroid.publish(centroid_msg)
                self.pub_error.publish(error_msg)
                self.pub_debug.publish(self.bridge.cv2_to_imgmsg(mask_clean, encoding='mono8'))
                return

            # Se achou a linha, calcula centróide ponderado do histograma do ROI
            col_sum = np.sum(roi > 0, axis=0).astype(np.float32)
            sum_w = np.sum(col_sum)

            xs = np.arange(w, dtype=np.float32)
            cx = float(np.sum(xs * col_sum) / sum_w) if sum_w > 0 else w / 2.0

            cy = (y0 + h) / 2.0

            center_x = w / 2.0
            error_px = cx - center_x
            error_norm = error_px / center_x  # normalizado approx entre -1 e 1

            # Filtro EMA para suavizar erro (evita oscilações bruscas)
            self.error_filtered = self.alpha * self.error_filtered + (1 - self.alpha) * error_norm
            error_norm_filtered = float(self.error_filtered)

            # Prepara mensagens para publicar
            centroid_msg.point.x = cx
            centroid_msg.point.y = cy
            centroid_msg.point.z = float(n_points)

            error_msg.vector.x = error_norm_filtered
            error_msg.vector.y = float(error_px)
            error_msg.vector.z = float(n_points)

            # Publica mensagens
            self.pub_centroid.publish(centroid_msg)
            self.pub_error.publish(error_msg)

            # Publica máscara limpa para debug visual
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(mask_clean, encoding='mono8'))

        except Exception as e:
            self.get_logger().error(f'VisionNode error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
