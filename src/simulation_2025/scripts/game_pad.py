#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)

        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        self.deadzone = 0.05  # Ölü bölge eşiği

    def apply_deadzone(self, value):
        """Joystick ölü bölgesini uygular."""
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def publish_velocity(self):
        pygame.event.pump()
        twist = Twist()

        # Sol joystick'in eksenlerini oku (İleri-Geri)
        linear_axis = self.joystick.get_axis(1)  # Sol joystick'in Y ekseni

        # Sağ joystick'in eksenlerini oku (Sağa-Sola Dönüş)
        angular_axis = self.joystick.get_axis(3)  # Sağ joystick'in X ekseni

        # Ölü bölgeyi uygula
        linear_axis = self.apply_deadzone(linear_axis)
        angular_axis = self.apply_deadzone(angular_axis)

        # Y eksenini ters çevir
        twist.linear.x = -linear_axis * 1.0  # Hızı ölçeklendirin
        twist.angular.z = -angular_axis * 1.0  # Açısal hızı ölçeklendirin

        self.publisher.publish(twist)
        self.get_logger().info(f'Published linear.x: {twist.linear.x}, angular.z: {twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    joystick_teleop = JoystickTeleop()

    try:
        rclpy.spin(joystick_teleop)
    except KeyboardInterrupt:
        pass

    joystick_teleop.destroy_node()
    rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()
