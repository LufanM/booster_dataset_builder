import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class TimeStampValidator(Node):
    def __init__(self):
        super().__init__('timestamp_validator')
        self.subscription = self.create_subscription(
            Image,
            '/camera_left/color/image_raw',
            self.image_callback,
            10
        )

        self.subscription_2 = self.create_subscription(
            Image,
            '/camera_right/color/image_raw',
            self.image_callback_2,
            10
        )
        self.get_logger().info('时间戳验证节点已启动...')
        self.current_time_left = 0.0
        self.current_time_right = 0.0
        self.msg_time_left = 0.0
        self.msg_time_right = 0.0


    def image_callback(self, msg):
        # 获取消息时间戳和当前系统时间
        self.msg_time_left = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.current_time_left = time.time()
        
        # 计算时间差
        time_diff = abs(self.current_time_left - self.msg_time_left)
        
        self.get_logger().info(f'消息时间: {self.msg_time_left:.6f}')
        self.get_logger().info(f'系统时间: {self.current_time_left:.6f}')
        self.get_logger().info(f'时间差: {time_diff:.6f}秒')
        
        # 如果时间差很小（例如小于0.1秒），说明正在使用系统时间
        if time_diff < 0.1:
            self.get_logger().info('✓ 正在使用系统时间')
        else:
            self.get_logger().warn('✗ 可能仍在使用设备时间')

    def image_callback_2(self, msg):
        # 获取消息时间戳和当前系统时间
        self.msg_time_right = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.current_time_right = time.time()

        self.get_logger().info(f'消息时间: {self.msg_time_right:.6f}')
        self.get_logger().info(f'系统时间: {self.current_time_right:.6f}')
        self.get_logger().info(f'左右手系统时间差: {(self.current_time_right - self.current_time_left):.6f}秒')
        self.get_logger().info(f'左右手消息时间差: {(self.msg_time_right - self.msg_time_left):.6f}秒\n')



def main(args=None):
    rclpy.init(args=args)
    node = TimeStampValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
