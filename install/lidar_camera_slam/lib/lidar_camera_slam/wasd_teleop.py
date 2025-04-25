#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios
import select
import threading

class WASDTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Movement parameters
        self.linear_speed = 0.5  # meters per second
        self.angular_speed = 1.0  # radians per second
        
        print("""
WASD Robot Control
------------------
W : move forward
S : move backward
A : turn left
D : turn right
SPACE : stop
Q : quit
""")

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        try:
            while True:
                key = self.getKey()
                
                # Create Twist message
                twist = Twist()
                
                if key == 'w':
                    twist.linear.x = self.linear_speed
                elif key == 's':
                    twist.linear.x = -self.linear_speed
                elif key == 'a':
                    twist.angular.z = self.angular_speed
                elif key == 'd':
                    twist.angular.z = -self.angular_speed
                elif key == ' ':
                    # Space bar to stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == 'q':
                    # Quit
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
                    break
                
                self.publisher.publish(twist)

        except Exception as e:
            print(e)

        finally:
            # Make sure to stop the robot when shutting down
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    teleop = WASDTeleop()
    
    # Create a thread for ROS spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop,))
    spin_thread.start()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    teleop.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main() 