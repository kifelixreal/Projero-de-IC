#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')
        
        # Publisher para /cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Lista de comandos para a rota (cada item: (linear_x, angular_z, duration))
        self.route = [
            (1.0, 0.0, 10.0),    # Ir reto  (aprox. 2m a 0.5m/s)
            (0.0, -1.0, 3.9),   # Virar direita  (aprox. 90 graus)
            
            (1.0, 0.0, 17.5),
            (0.0, -1.0, 3.8),
            
            (1.0, 0.0, 7.0),
            (0.0, -1.0, 3.9),
            
            (1.0, 0.0, 2.0),
            (0.0, -1.0, 3.9),
            
            (1.0, 0.0, 2.5),
            (0.0, 0.0, 1.0)     # Parar 
        ]
        
        self.current_command_index = 0
        self.start_time = None
        self.cmd = Twist()
        
        # Timer para publicar a 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Nó de navegação autônoma iniciado. Iniciando rota em 2 segundos...')
        time.sleep(2)  # Pausa inicial para o robô se preparar
        self.execute_route()

    def execute_route(self):
        if self.current_command_index < len(self.route):
            linear_x, angular_z, duration = self.route[self.current_command_index]
            self.cmd.linear.x = linear_x
            self.cmd.angular.z = angular_z
            self.start_time = time.time()
            self.get_logger().info(f'Executando comando {self.current_command_index + 1}: linear={linear_x}, angular={angular_z}, duração={duration}s')
        else:
            # Fim da rota
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher.publish(self.cmd)
            self.get_logger().info('Rota concluída! Parando o robô.')
            self.timer.cancel()  # Para o timer

    def timer_callback(self):
        if self.start_time is not None:
            elapsed = time.time() - self.start_time
            linear_x, angular_z, duration = self.route[self.current_command_index]
            
            if elapsed >= duration:
                # Comando terminado, próximo
                self.current_command_index += 1
                self.start_time = None
                self.execute_route()
            else:
                # Publica o comando atual
                self.publisher.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    navigator = AutonomousNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

