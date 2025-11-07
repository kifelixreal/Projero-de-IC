#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')

        # Publisher para comandos de velocidade
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber para odometria
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        # Estado do robô
        self.state = 'idle'  # 'idle', 'moving_linear', 'rotating'

        # Variáveis de posição e orientação
        self.x_current = 0.0
        self.y_current = 0.0
        self.yaw_current = 0.0

        # Variáveis para controle de movimento
        self.x_init = None
        self.y_init = None

        # Variáveis para controle de giro incremental
        self.yaw_last = None
        self.yaw_accumulated = 0.0

        # Comando Twist atual
        self.cmd = Twist()

        # Índice do comando atual na rota
        self.current_command_index = 0

        # Rota: (vel_linear, vel_angular, dist_linear, ang_angular)
        self.route = [
            (0.5, 0.0, 4.5, 0.0),       # 
            (0.0, -0.2, 0.0, -math.pi/2), # 
            
            (0.5, 0.0, 7.5, 0.0),       # 
            (0.0, -0.2, 0.0, -math.pi/2), #
            
            (0.5, 0.0, 3.5, 0.0),
            (0.0, -0.2, 0.0, -math.pi/2), #
            
            (0.5, 0.0, 2.0, 0.0),
            (0.0, -0.2, 0.0, -math.pi/2), #
            
            (0.5, 0.0, 1.0, 0.0),
            
        ]

        # Timer para controle a 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Nó iniciado, aguardando odometria...')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg):
        # Atualiza posição
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y

        # Extrai yaw do quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_current = math.atan2(siny_cosp, cosy_cosp)

        # Inicializa yaw_last na primeira leitura
        if self.yaw_last is None:
            self.yaw_last = yaw_current

        # Calcula delta incremental e acumula se estiver girando
        delta = self.normalize_angle(yaw_current - self.yaw_last)
        if self.state == 'rotating':
            self.yaw_accumulated += delta

        self.yaw_last = yaw_current
        self.yaw_current = yaw_current

    def distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def timer_callback(self):
        if self.state == 'idle':
            if self.current_command_index >= len(self.route):
                # Rota concluída
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher.publish(self.cmd)
                self.get_logger().info('Rota concluída. Parando.')
                self.timer.cancel()
                return

            # Pega comando atual
            linear_vel, angular_vel, linear_dist, angular_dist = self.route[self.current_command_index]

            # Configura comando Twist
            self.cmd.linear.x = linear_vel
            self.cmd.angular.z = angular_vel

            # Salva posição inicial para controle linear
            self.x_init = self.x_current
            self.y_init = self.y_current

            # Reseta acumulador de giro se for rotação
            if abs(linear_dist) == 0 and abs(angular_dist) > 0:
                self.state = 'rotating'
                self.yaw_accumulated = 0.0
                self.get_logger().info(f'Iniciando giro: alvo {math.degrees(angular_dist):.1f}°')
            elif abs(linear_dist) > 0:
                self.state = 'moving_linear'
                self.get_logger().info(f'Iniciando movimento linear: alvo {linear_dist:.2f} m')
            else:
                # Comando vazio, pula para próximo
                self.current_command_index += 1
                return

            self.publisher.publish(self.cmd)

        elif self.state == 'moving_linear':
            dist_moved = self.distance(self.x_init, self.y_init, self.x_current, self.y_current)
            target_dist = self.route[self.current_command_index][2]

            if dist_moved >= target_dist:
                # Parar movimento linear
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher.publish(self.cmd)
                self.state = 'idle'
                self.current_command_index += 1
                self.get_logger().info(f'Movimento linear concluído: {dist_moved:.2f} m')
            else:
                self.publisher.publish(self.cmd)

        elif self.state == 'rotating':
            target_angle = self.route[self.current_command_index][3]
            tolerance = 0.035  # ~2 graus em radianos

            self.get_logger().info(f'Giro acumulado: {math.degrees(self.yaw_accumulated):.2f}°, alvo: {math.degrees(target_angle):.2f}°')

            # Verifica se atingiu o ângulo alvo considerando sentido e tolerância
            if (target_angle > 0 and self.yaw_accumulated >= target_angle - tolerance) or \
               (target_angle < 0 and self.yaw_accumulated <= target_angle + tolerance):
                # Parar giro
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher.publish(self.cmd)
                self.state = 'idle'
                self.current_command_index += 1
                self.get_logger().info('Giro concluído')
            else:
                self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    navigator = AutonomousNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Parando por Ctrl+C...')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

