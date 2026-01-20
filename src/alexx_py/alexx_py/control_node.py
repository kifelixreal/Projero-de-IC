import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped, PointStamped, Twist


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # -------------------- Parâmetros principais --------------------
        # PID (para erro NORMALIZADO em vector.x, tipicamente ~[-1, 1])
        self.kp = 1.2
        self.ki = 0.1
        self.kd = 0.08

        # Anti-windup (limite da integral)
        self.max_integral = 0.6

        # Limites de velocidade
        self.max_angular = 0.35          # rad/s
        self.v_min = 0.03               # m/s (quando erro grande)
        self.v_max = 0.12               # m/s (quando erro pequeno)

        # Deadband: se erro for muito pequeno, zera (evita tremedeira)
        self.deadband = 0.02

        # Confiança mínima (vector.z) para considerar linha válida
        self.min_confidence = 80.0

        # "Baixo_lines": só aceite a linha se o centróide estiver na parte de baixo da imagem
        # Como o PointStamped.point.y está em pixels, você precisa saber a altura da imagem.
        # Se não souber, deixe essa validação desligada (use False).
        self.use_baixo_lines_check = False
        self.image_height_px = 480.0          # ajuste se sua câmera não for 480
        self.min_centroid_y_ratio = 0.60      # centróide deve estar abaixo de 60% da altura

        # Se perder linha: procurar por alguns segundos; se não achar, parar (fim do percurso)
        self.search_when_lost = True
        self.search_angular = 0.18
        self.lost_timeout_sec = 2.0           # tempo sem linha para considerar "fim"

        # Inverter sentido do giro se necessário
        self.invert_steering = True

        # -------------------- Estado --------------------
        self.error_norm = 0.0
        self.confidence = 0.0
        self.last_seen_time = time.time()

        self.centroid_y = 999999.0  # último y recebido do centróide (pixels)

        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_t = time.time()

        # -------------------- Sub/Pub --------------------
        self.sub_error = self.create_subscription(
            Vector3Stamped,
            '/line_error',
            self.error_cb,
            10
        )

        # Opcional: usar centróide para validar baixo_lines
        self.sub_centroid = self.create_subscription(
            PointStamped,
            '/line_centroid',
            self.centroid_cb,
            10
        )

        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Loop de controle em 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('ControlNode: PID seguindo linha via /line_error e /line_centroid')

    def error_cb(self, msg: Vector3Stamped):
        # erro normalizado (mais adequado pro PID)
        self.error_norm = float(msg.vector.x)

        # confiança (quanto maior, mais seguro)
        self.confidence = float(msg.vector.z)

        # se a confiança é suficiente, consideramos que "viu a linha agora"
        if self.confidence >= self.min_confidence:
            self.last_seen_time = time.time()

    def centroid_cb(self, msg: PointStamped):
        # y do centróide em pixels (útil para garantir que estamos vendo a parte de baixo)
        self.centroid_y = float(msg.point.y)

    def line_is_valid(self) -> bool:
        """Decide se a linha está válida para controle neste instante."""
        # Checagem 1: confiança do vision_node
        if self.confidence < self.min_confidence:
            return False

        # Checagem 2 (opcional): baixo_lines, centróide deve estar na parte inferior
        if self.use_baixo_lines_check:
            if self.image_height_px <= 0:
                return False
            y_ratio = self.centroid_y / self.image_height_px
            if y_ratio < self.min_centroid_y_ratio:
                return False

        return True

    def control_loop(self):
        now = time.time()

        # dt do PID
        dt = now - self.prev_t
        if dt <= 0.0:
            dt = 1e-3
        self.prev_t = now

        twist = Twist()

        # --------- Caso: linha não válida (perdeu) ---------
        if not self.line_is_valid():
            lost_for = now - self.last_seen_time

            # Se perdeu por pouco tempo: tenta procurar girando
            if self.search_when_lost and lost_for < self.lost_timeout_sec:
                twist.linear.x = 0.0
                twist.angular.z = self.search_angular
                self.pub_cmd.publish(twist)
                return

            # Se perdeu por tempo demais: considera fim do percurso e para
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd.publish(twist)

            # Reseta PID (importante)
            self.integral = 0.0
            self.prev_error = 0.0

            return

        # --------- Linha válida: aplica PID ---------
        error = self.error_norm

        # deadband
        if abs(error) < self.deadband:
            error = 0.0

        # integral com anti-windup
        self.integral += error * dt
        self.integral = clamp(self.integral, -self.max_integral, self.max_integral)

        # derivada
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # PID
        u = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        # limita giro
        u = clamp(u, -self.max_angular, self.max_angular)

        # inverter se estiver virando pro lado errado
        if self.invert_steering:
            u = -u

        # --------- Speed scheduling (anda menos se erro grande) ---------
        # erro_abs ~ 0 (centrado) -> v_max
        # erro_abs ~ 1 (muito fora) -> v_min
        e = min(1.0, abs(error))
        v = self.v_min + (self.v_max - self.v_min) * (1.0 - e)

        twist.linear.x = float(v)
        twist.angular.z = float(u)
        self.pub_cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
