import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class TimeStampValidator(Node):
    def __init__(self):
        super().__init__('timestamp_validator')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info('时间戳验证节点已启动...')

    def image_callback(self, msg):
        # 获取消息时间戳和当前系统时间
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        current_time = time.time()
        
        # 计算时间差
        time_diff = abs(current_time - msg_time)
        
        self.get_logger().info(f'消息时间: {msg_time:.6f}')
        self.get_logger().info(f'系统时间: {current_time:.6f}')
        self.get_logger().info(f'时间差: {time_diff:.6f}秒')
        
        # 如果时间差很小（例如小于0.1秒），说明正在使用系统时间
        if time_diff < 0.1:
            self.get_logger().info('✓ 正在使用系统时间')
        else:
            self.get_logger().warn('✗ 可能仍在使用设备时间')

def main(args=None):
    rclpy.init(args=args)
    node = TimeStampValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()