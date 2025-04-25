#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import time

msg = """
Control Your Robot!
------------------
Moving around:
   w
a  s  d

w/s : move forward/backward
a/d : turn left/right
SPACE : force stop
q : quit
"""

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.last_key = None
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Movement parameters
        self.linear_speed = 0.5  # meters per second
        self.angular_speed = 1.0  # radians per second
        
        print(msg)
        print("Waiting for key input...")
        print(f"Publishing to topic: {self.publisher.topic_name}")

    def getKey(self):
        try:
            tty.setraw(sys.stdin.fileno())
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_callback(self):
        # Create and publish Twist message based on last key press
        twist = Twist()
        
        if self.last_key == 'w':
            twist.linear.x = self.linear_speed
            print(f"Moving forward at {self.linear_speed} m/s")
        elif self.last_key == 's':
            twist.linear.x = -self.linear_speed
            print(f"Moving backward at {self.linear_speed} m/s")
        elif self.last_key == 'a':
            twist.angular.z = self.angular_speed
            print(f"Turning left at {self.angular_speed} rad/s")
        elif self.last_key == 'd':
            twist.angular.z = -self.angular_speed
            print(f"Turning right at {self.angular_speed} rad/s")
        elif self.last_key == ' ':
            print("Stopping robot")
        
        self.publisher.publish(twist)
        print(f"Published command: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

    def run(self):
        try:
            while True:
                key = self.getKey()
                if key == '\x03':  # CTRL+C
                    break
                
                # Update last key
                if key in ['w', 'a', 's', 'd', ' ']:
                    self.last_key = key
                    if key == ' ':  # Space to stop
                        self.last_key = None
                    print(f"Received key: {key}")
                elif key == 'q':
                    print("Quitting...")
                    break

        except Exception as e:
            print(f"Error: {e}")

        finally:
            # Stop the robot
            twist = Twist()
            self.publisher.publish(twist)
            print("Stopping robot and cleaning up...")
            
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
    try:
        controller.run()
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    finally:
        # Stop the robot
        twist = Twist()
        controller.publisher.publish(twist)
        print("Final cleanup...")
        
        # Cleanup
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 