#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Mover(Node):
    def __init__(self):
        super().__init__('mover_node')
        # /cmd_vel トピック用のパブリッシャーを作成
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # 0.5秒ごとに move_robot メソッドを呼び出すタイマーを作成
        self.timer_ = self.create_timer(0.5, self.move_robot)
        self.get_logger().info('Mover node has been started.')

    def move_robot(self):
        # Twist メッセージを作成
        twist = Twist()
        # 前進するための速度を設定
        twist.linear.x = 0.2  # 0.2 m/sで前進
        twist.angular.z = 0.0 # 回転はしない

        # メッセージをパブリッシュ
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing: linear.x={twist.linear.x}')

def main(args=None):
    rclpy.init(args=args)
    mover_node = Mover()
    try:
        rclpy.spin(mover_node)
    except KeyboardInterrupt:
        pass
    finally:
        mover_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
