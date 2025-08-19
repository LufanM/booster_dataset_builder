from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber, B1LocoClient, Frame, Transform

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import time
import cv2
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R
import pinocchio as pin

class ArmKinematicsSolver:
    def __init__(self, urdf_path):
        # 加载URDF模型
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # 获取手臂末端执行器帧ID
        self.left_ee_id = self.model.getFrameId("left_arm_end_effector")
        self.right_ee_id = self.model.getFrameId("right_arm_end_effector")
        
        # 预分配内存
        self.q = pin.neutral(self.model)
        self.left_ee_pose = pin.SE3.Identity()
        self.right_ee_pose = pin.SE3.Identity()
    
    def compute_arm_poses(self, joint_positions):
        start_time = time.time()
        for i, pos in enumerate(joint_positions):
            self.q[i] = pos  # 如果使用浮动基座, 前7个是基座自由度(需要+7)
        
        # 计算正向运动学
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        self.left_ee_pose = self.data.oMf[self.left_ee_id]
        self.right_ee_pose = self.data.oMf[self.right_ee_id]
        
        # 转换为更易处理的格式
        left_pos = self.left_ee_pose.translation
        left_rot = pin.Quaternion(self.left_ee_pose.rotation).coeffs()  # [x,y,z,w]   
        right_pos = self.right_ee_pose.translation
        right_rot = pin.Quaternion(self.right_ee_pose.rotation).coeffs()
        
        print(f"left position: {left_pos}, orientation: {left_rot}\n"
              f"right position: {right_pos}, orientation: {right_rot}")
        
class DualCameraSubscriber(Node):
    def __init__(self, cv_show=False):
        super().__init__('synch_camera_subscriber')
        self.get_logger().info("相机同步订阅节点启动...")
        
        # 消息计数器
        self.message_count = 0
        self.start_time = time.time()
        self.cv_render_flag = cv_show
        self.last_frame_time = time.time()  # 上一帧时间

        # 创建相机订阅器
        # self.head_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.left_sub = Subscriber(self, Image, '/camera_left/color/image_raw')
        self.right_sub = Subscriber(self, Image, '/camera_right/color/image_raw')
        
        self.get_logger().info("已订阅相机话题:")
        
        # 创建时间同步器
        self.sync = ApproximateTimeSynchronizer(
            # [self.head_sub, self.left_sub, self.right_sub],
            [self.left_sub, self.right_sub],
            queue_size=20,
            slop=0.1  # 允许100ms的时间差
        )
        self.sync.registerCallback(self.sync_callback)
        self.get_logger().info("时间同步器已配置 (队列大小=20, 时间差容限=0.1s)")
        
        # 创建窗口用于显示同步结果
        if self.cv_render_flag:
            cv2.namedWindow("Synchronized Stereo Cameras", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Synchronized Stereo Cameras", 1200, 400)
        
        # 性能监控
        self.max_diff_history = []
        self.avg_delay_history = []
        
        self.get_logger().info("节点已启动，等待相机消息同步...")

    def manual_image_conversion(self, msg):
        """手动转换图像（不使用 cv_bridge）"""
        try:
            if msg.encoding == 'bgr8':
                # 直接创建 OpenCV 图像
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                # 创建 RGB 图像
                rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                # 转换为 BGR
                cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().error(f"不支持的编码格式: {msg.encoding}")
                return None
            
            # self.get_logger().info("手动图像转换成功")
            return cv_image
        except Exception as e:
            self.get_logger().error(f"手动图像转换失败: {str(e)}")
            return None
        
    # def sync_callback(self, head_msg, left_msg, right_msg):
    def sync_callback(self, left_msg, right_msg):
        """处理同步后的相机帧"""
        self.message_count += 1
        current_time = time.time()

        # 计算帧率
        frame_interval = current_time - self.last_frame_time
        current_fps = 1.0 / frame_interval if frame_interval > 0 else 0
        self.last_frame_time = current_time
            
        # 每30帧打印一次信息
        if self.message_count % 30 == 0:  
            self.get_logger().info(f"\n=== 帧 #{self.message_count} | 帧率: {current_fps:.2f} Hz ===")
            # self.get_logger().info(f"收到头 图像: 宽度={head_msg.width}, 高度={head_msg.height}, 编码={head_msg.encoding}")
            self.get_logger().info(f"收到左 图像: 宽度={left_msg.width}, 高度={left_msg.height}, 编码={left_msg.encoding}")
            self.get_logger().info(f"收到右 图像: 宽度={right_msg.width}, 高度={right_msg.height}, 编码={right_msg.encoding}")

        # 计算时间戳
        # head_timestamp = head_msg.header.stamp.sec + head_msg.header.stamp.nanosec/1e9
        left_timestamp = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec/1e9
        right_timestamp = right_msg.header.stamp.sec + right_msg.header.stamp.nanosec/1e9
        
        # 计算图像之间的时间差
        time_diff = abs(left_timestamp - right_timestamp)
        # time_diff_head = abs(head_timestamp - right_timestamp) 
        
        # 计算系统延迟
        left_delay = current_time - left_timestamp
        right_delay = current_time - right_timestamp
        # head_delay = current_time - head_timestamp
        avg_delay = (left_delay + right_delay) / 2.0
        
        # 记录性能数据
        self.max_diff_history.append(time_diff)
        self.avg_delay_history.append(avg_delay) # 后续没必要可以关闭
        
        # 每10帧打印一次信息 关于延迟
        if self.message_count % 10 == 0:
            avg_diff = sum(self.max_diff_history[-10:]) / 10.0
            avg_delay = sum(self.avg_delay_history[-10:]) / 10.0
            
            self.get_logger().info(
                f"同步消息 #{self.message_count}:\n"
                f"  图像当前时间差: {time_diff*1000:.1f}ms\n"
                f"  平均时间差: {avg_diff*1000:.1f}ms\n"
                f"  平均系统延迟: {avg_delay*1000:.1f}ms\n"
                # f"  头部图像当前时间差：{time_diff_head*1000:.1f}ms\n"
                # f"  头部图像系统延迟：{head_delay*1000:.1f}ms"
            )

        # head_img = self.manual_image_conversion(head_msg)
        left_img = self.manual_image_conversion(left_msg)
        right_img = self.manual_image_conversion(right_msg)
        # 转换图像为OpenCV格式
        if self.cv_render_flag:
            try:
                # 调整图像大小以匹配显示
                def resize_img(img, max_height=400):
                    h, w = img.shape[:2]
                    scale = max_height / h
                    return cv2.resize(img, (int(w * scale), int(h * scale)))
                
                left_img = resize_img(left_img)
                right_img = resize_img(right_img)
                
                # 创建并排视图
                combined = np.hstack((left_img, right_img))
                
                # 添加相机标识
                cv2.putText(combined, "Left Camera", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined, "Right Camera", (left_img.shape[1] + 10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # 添加分隔线
                cv2.line(combined, (left_img.shape[1], 0), 
                        (left_img.shape[1], left_img.shape[0]), (0, 255, 0), 2)
                
                # 显示时间同步信息
                sync_info = f"Frame #{self.message_count} | Image Diff: {time_diff*1000:.1f}ms | Delay: {avg_delay*1000:.1f}ms"
                cv2.putText(combined, sync_info, (10, combined.shape[0] - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # 添加时间差指示器（颜色编码）
                color = (0, 200, 0) if time_diff < 0.05 else (0, 165, 255) if time_diff < 0.1 else (0, 0, 255)
                cv2.rectangle(combined, (10, combined.shape[0] - 40), 
                            (int(10 + 300 * min(time_diff*10, 1.0)), combined.shape[0] - 30), 
                            color, -1)
                cv2.putText(combined, "Time Diff Indicator", (10, combined.shape[0] - 45), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                # 显示图像
                cv2.imshow("Synchronized Stereo Cameras", combined)
                cv2.waitKey(1)
                
            except Exception as e:
                self.get_logger().error(f"图像处理错误: {str(e)}")

    def print_stats(self):
        """打印统计信息"""
        elapsed = time.time() - self.start_time
        rate = self.message_count / elapsed if elapsed > 0 else 0
        
        if self.max_diff_history:
            avg_diff = sum(self.max_diff_history) / len(self.max_diff_history)
            max_diff = max(self.max_diff_history)
            min_diff = min(self.max_diff_history)
        else:
            avg_diff = max_diff = min_diff = 0.0
            
        if self.avg_delay_history:
            avg_delay = sum(self.avg_delay_history) / len(self.avg_delay_history)
        else:
            avg_delay = 0.0
            
        self.get_logger().info(
            f"\n===== 统计信息 =====\n"
            f" 运行时间: {elapsed:.1f}秒\n"
            f" 同步消息组数: {self.message_count}\n"
            f" 平均同步率: {rate:.1f} 组/秒\n"
            f" 时间差统计:\n"
            f"   平均: {avg_diff*1000:.1f}ms\n"
            f"   最大: {max_diff*1000:.1f}ms\n"
            f"   最小: {min_diff*1000:.1f}ms\n"
            f" 平均系统延迟: {avg_delay*1000:.1f}ms\n"
            f"====================\n"
        )

    def __del__(self):
        cv2.destroyAllWindows()

class B1DataProcessor:
    def __init__(self, urdf_path):
        # 初始化运动学求解器
        self.kinematics_solver = ArmKinematicsSolver(urdf_path)

        self.frame_count = 0
        self.start_time = time.time()
        self.last_print_time = time.time()
        self.fps = 0.0

    def handler(self, low_state_msg):
        # self.kinematics_solver.compute_arm_poses()
        """处理接收到的低状态消息并控制打印频率"""
        self.frame_count += 1

        # 计算帧率（每100帧更新一次，避免波动过大）
        if self.frame_count % 100 == 0:
            current_time = time.time()
            elapsed = current_time - self.start_time
            self.fps = self.frame_count / elapsed
            
        # 每10帧打印一次数据
        if self.frame_count % 250 == 0:
            self.print_data(low_state_msg)

    def print_data(self, low_state_msg):
        """格式化打印机器人状态数据"""
        print(f"\n=== 帧 #{self.frame_count} | 帧率: {self.fps:.2f} Hz ===")
        
        print(f"  串口电机数量: {len(low_state_msg.motor_state_serial)}")
        # print(f"  串口电机数量: {low_state_msg.motor_state_serial}")
        print(f"  并行电机数量: {len(low_state_msg.motor_state_parallel)}")
        
        imu_state = low_state_msg.imu_state
        print(f"  IMU: 滚转={imu_state.rpy[0]:.2f}, 俯仰={imu_state.rpy[1]:.2f}, 偏航={imu_state.rpy[2]:.2f}")
        print(f"       角速度=[{imu_state.gyro[0]:.2f}, {imu_state.gyro[1]:.2f}, {imu_state.gyro[2]:.2f}]")
        print(f"       加速度=[{imu_state.acc[0]:.2f}, {imu_state.acc[1]:.2f}, {imu_state.acc[2]:.2f}]")
        
        # 打印前3个串口电机数据（避免过多输出）
        for i, motor in enumerate(low_state_msg.motor_state_serial[2:9]):
            print(f"  串口电机 {i}: 位置={motor.q:.2f}, 速度={motor.dq:.2f}, 加速度={motor.ddq:.2f}, 估计力矩={motor.tau_est:.2f}")
        
        # 打印前3个并行电机数据
        for i, motor in enumerate(low_state_msg.motor_state_parallel[:3]):
            print(f"  并行电机 {i}: 速度={motor.dq:.2f}, 加速度={motor.ddq:.2f}, 估计力矩={motor.tau_est:.2f}")
        
# 创建并启动 ROS 节点线程
def run_ros_node():
    rclpy.init()
    node = DualCameraSubscriber(cv_show=False)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("键盘中断，关闭节点...")
        node.print_stats()
    except Exception as e:  
        node.get_logger().error(f"节点运行时出错: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    URDF_PATH = "./T1_7_dof_arm_serial_with_head_arm.urdf"

    # 创建并启动所有线程
    threads = []
    
    # ROS节点线程
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.daemon = True
    ros_thread.start()
    threads.append(ros_thread)

    # # 状态处理线程
    ChannelFactory.Instance().Init(0)
    processor = B1DataProcessor(URDF_PATH)
    channel_subscriber = B1LowStateSubscriber(processor.handler)
    channel_subscriber.InitChannel()
    
    try:
        # 主线程保持运行，等待退出信号
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭程序...")


if __name__ == "__main__":
    main()
