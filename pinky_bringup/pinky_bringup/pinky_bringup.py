import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from .pinkylib import Pinky

class MotorControlNode(Node):
 
    def __init__(self):
        super().__init__('pinky_bringup')
 
        self.pinky = Pinky()

        self.pinky.enable_motor()
        self.pinky.start_motor()
 
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
 
    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x 
        angular_z = msg.angular.z / 5

        # 좌우 회전
        left_speed = linear_x - angular_z 
        right_speed = linear_x + angular_z

        set_l = self.custom_map(left_speed)
        set_r = self.custom_map(right_speed)
 
        self.pinky.move(set_l, set_r)

    def custom_map(self, value):
        if value == 0:
            return 0
        elif value > 0:
            return 25 + ((value * 20) / 0.5)
        else:
            return -25 + ((value * 20) / 0.5)

    def destroy_node(self):
        # PWM 정지 및 GPIO 정리
        self.pinky.disable_motor()
        self.pinky.stop_motor()
        self.pinky.clean()
        
def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()
     
    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()
 
if __name__ == '__main__':
    main()