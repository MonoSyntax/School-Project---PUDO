#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select

import tty, termios
  
msg = """
Control OTAGG Car!
---------------------------
Moving around:
        w
   a    s    d
        x

s : reverse
x : hand brake; just stop

CTRL-C to quit
"""

MAX_ANGULAR_SPEED = 5.0
MAX_LINEAR_SPEED = 30.0

class CarControlNode(Node): 
    def __init__(self):
        super().__init__("cmd_vel") 
        
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel",10)
        msg = Twist()
              
        while True:
            self.settings = termios.tcgetattr(sys.stdin)
            key = self.getKey()

            if key == "w":
                msg.linear.x += 0.5
                msg.linear.x = min(MAX_LINEAR_SPEED, msg.linear.x)
            elif key == "a":
                if msg.linear.x<0:
                    msg.angular.z -= 0.05
                else:
                    msg.angular.z += 0.05
                msg.angular.z = min(MAX_ANGULAR_SPEED, msg.angular.z)
            elif key == "s":
                msg.linear.x -= 0.5
                msg.linear.x = max(-MAX_LINEAR_SPEED, msg.linear.x)
            elif key == "d":
                if msg.linear.x<0:
                    msg.angular.z += 0.05
                else:
                    msg.angular.z -= 0.05
                msg.angular.z = max(-MAX_ANGULAR_SPEED, msg.angular.z)
            elif key == "x":
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                if key == "\x03":
                    break

            if msg.linear.x==0.0:
                msg.angular.z=0.0

            self.publisher_.publish(msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


def main(args=None):
    print(msg)
    rclpy.init(args=args)
    node = CarControlNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
