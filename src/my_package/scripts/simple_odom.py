#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math
from turtlesim.msg import Pose

class SimpleOdom(Node):
    def __init__(self):
        super().__init__('simple_odom_node')
        # ロボットの現在の姿勢（位置と向き）
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        
        # 最後にメッセージを受け取った時間
        self.last_time_ = self.get_clock().now()

        # TransformBroadcasterの初期化
        self.tf_broadcaster_ = TransformBroadcaster(self)

        # /cmd_velトピックのSubscriberを作成
        self.subscription_ = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
        self.get_logger().info('Simple Odom node has been started.')

    def cmd_vel_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time_).nanoseconds / 1e9  # 経過時間を秒に変換

        # 速度指令に基づいて移動量を計算
        self.x_ += msg.linear.x * math.cos(self.theta_) * dt
        self.y_ += msg.linear.x * math.sin(self.theta_) * dt
        self.theta_ += msg.angular.z * dt

        self.last_time_ = current_time

        # TransformStampedメッセージを作成
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'  # 親フレーム
        t.child_frame_id = 'base_link' # 子フレーム
        
        # 位置をセット
        t.transform.translation.x = self.x_
        t.transform.translation.y = self.y_
        t.transform.translation.z = 0.0

        # 向き（クォータニオン）をセット
        q = quaternion_from_euler(0, 0, self.theta_)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # TFをブロードキャスト
        self.tf_broadcaster_.sendTransform(t)

def quaternion_from_euler(roll, pitch, yaw):
    """
    オイラー角からクォータニオンに変換する補助関数
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr  # x
    q[1] = sy * cp * sr + cy * sp * cr  # y
    q[2] = sy * cp * cr - cy * sp * sr  # z
    q[3] = cy * cp * cr + sy * sp * sr  # w

    return q

def main(args=None):
    rclpy.init(args=args)
    simple_odom_node = SimpleOdom()
    rclpy.spin(simple_odom_node)
    simple_odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
