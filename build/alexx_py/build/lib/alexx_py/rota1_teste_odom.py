#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')  # Inicializa o nó ROS 2 com nome 'autonomous_navigator'

        # Publisher para enviar comandos de velocidade para o tópico /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber para receber mensagens de odometria do tópico /odom
        self.subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 30)

        # Variáveis para armazenar a posição e orientação atuais do robô
        self.x_current = 0.0
        self.y_current = 0.0
        self.yaw_current = 0.0

        # Variáveis para armazenar a posição e orientação no início do comando atual
        self.x_init = None
        self.y_init = None
        self.yaw_init = None

        # Mensagem Twist que será publicada para controlar o robô
        self.cmd = Twist()

        # Índice do comando atual na lista de comandos da rota
        self.current_command_index = 0

        # Estado do controlador: 'idle' (parado), 'moving_linear' (andando em linha reta), 'rotating' (girando)
        self.state = 'idle'

        # Lista de comandos da rota
        # Cada comando é uma tupla: (velocidade_linear, velocidade_angular, distância_linear, ângulo_angular)
        # Distância em metros, ângulo em radianos
        self.route = [
            (1.0, 0.0, 4.5, 0.0),       # 
            (0.0, -0.4, 0.0, math.pi/2), # 
            
            (1.0, 0.0, 8.0, 0.0),       # 
            (0.0, -0.4, 0.0, math.pi/2), #
            
            (1.0, 0.0, 6.5, 0.0),
            (0.0, -0.4, 0.0, math.pi/2), #
            
            (1.0, 0.0, 3.0, 0.0),
            (0.0, -0.4, 0.0, math.pi/2), #
            
            (1.0, 0.0, 2.0, 0.0),
            
        
        ]

        # Timer que chama a função timer_callback a cada 0.1 segundos (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('Nó iniciado, aguardando odometria...')

    def odom_callback(self, msg):
        # Callback chamado sempre que chega uma mensagem de odometria

        # Atualiza a posição atual do robô
        self.x_current = msg.pose.pose.position.x
        self.y_current = msg.pose.pose.position.y

        # Extrai a orientação em yaw (ângulo no plano XY) a partir do quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw_current = math.atan2(siny_cosp, cosy_cosp)

    def distance(self, x1, y1, x2, y2):
        # Calcula a distância euclidiana entre dois pontos (x1,y1) e (x2,y2)
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def normalize_angle(self, angle):
        # Normaliza um ângulo para o intervalo [-pi, pi]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle



    def timer_callback(self):
        # Função chamada periodicamente para controlar o movimento do robô

        if self.state == 'idle':
            # Se não há comando em execução, inicia o próximo comando da rota

            if self.current_command_index >= len(self.route):
                # Se todos os comandos foram executados, para o robô e cancela o timer
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher.publish(self.cmd)
                self.get_logger().info('Rota concluída. Parando.')
                self.timer.cancel()
                return

            # Obtém o comando atual da rota
            linear_vel, angular_vel, linear_dist, angular_dist = self.route[self.current_command_index]

            # Define as velocidades lineares e angulares para o comando atual
            self.cmd.linear.x = linear_vel
            self.cmd.angular.z = angular_vel

            # Salva a posição e orientação atuais para medir o deslocamento durante o comando
            self.x_init = self.x_current
            self.y_init = self.y_current
            self.yaw_init = self.yaw_current

            # Define o estado conforme o tipo de movimento do comando
            if linear_dist > 0:
                self.state = 'moving_linear'  # Movimento em linha reta
            elif abs(angular_dist) > 0:
                self.state = 'rotating'       # Movimento de rotação
            else:
                # Se o comando não tem movimento, passa para o próximo comando
                self.current_command_index += 1
                return

            self.get_logger().info(f'Executando comando {self.current_command_index + 1}: linear {linear_dist}m, angular {angular_dist}rad')

            # Publica o comando para iniciar o movimento
            self.publisher.publish(self.cmd)

        elif self.state == 'moving_linear':
            # Durante o movimento em linha reta, verifica se a distância desejada foi percorrida

            dist_moved = self.distance(self.x_init, self.y_init, self.x_current, self.y_current)

            if dist_moved >= self.route[self.current_command_index][2]:
                # Se a distância foi atingida, para o robô e muda para estado idle para próximo comando
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher.publish(self.cmd)
                self.state = 'idle'
                self.current_command_index += 1
            else:
                # Continua publicando o comando de movimento
                self.publisher.publish(self.cmd)

        elif self.state == 'rotating':
            # Durante a rotação, verifica se o ângulo desejado foi alcançado

            delta_yaw = self.normalize_angle(self.yaw_current - self.yaw_init)  # Ângulo girado desde o início do comando
            target_angle = self.route[self.current_command_index][3]           # Ângulo alvo do comando
            tolerance = 0.035  # ~2 graus em radianos
            
            #self.get_logger().info(f'Giro atual: {math.degrees(delta_yaw):.2f}°, alvo: {math.degrees(target_angle):.2f}°')
            self.get_logger().info(f'yaw_init: {math.degrees(self.yaw_init):.2f}°, yaw_current: {math.degrees(self.yaw_current):.2f}°, delta_yaw: {math.degrees(delta_yaw):.2f}°')
		
            if (target_angle > 0 and delta_yaw >= target_angle - tolerance) or \
   (target_angle < 0 and delta_yaw <= target_angle + tolerance):
                # Se o ângulo foi atingido, para o robô e muda para estado idle para próximo comando
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.publisher.publish(self.cmd)
                self.state = 'idle'
                self.current_command_index += 1
            else:
                # Continua publicando o comando de rotação
                self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)  # Inicializa o ROS 2
    navigator = AutonomousNavigator()  # Cria o nó de navegação autônoma

    try:
        rclpy.spin(navigator)  # Mantém o nó rodando até interrupção
    except KeyboardInterrupt:
        navigator.get_logger().info('Parando por Ctrl+C...')
    finally:
        navigator.destroy_node()  # Destrói o nó ao finalizar
        rclpy.shutdown()          # Encerra o ROS 2

if __name__ == '__main__':
    main()

