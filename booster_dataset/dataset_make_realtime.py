"""还没有Debug"""

import os
import time
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import ApproximateTimeSynchronizer, Subscriber
import pinocchio as pin
from booster_robotics_sdk_python import ChannelFactory, B1LowStateSubscriber
import collections

class DataCollector:
    def __init__(self, save_dir="data", episode_length=100):
        urdf_path = "./T1_7_dof_arm_serial_with_head_arm.urdf"
        self.kinematics_solver = ArmKinematicsSolver(urdf_path)

        self.save_dir = save_dir
        self.episode_length = episode_length
        self.episodes = []
        self.current_episode = []
        self.episode_count = 0
        self.step_count = 0
        self.running = True

        # 创建保存目录
        os.makedirs(save_dir, exist_ok=True)
        os.makedirs(os.path.join(save_dir, "train"), exist_ok=True)
        os.makedirs(os.path.join(save_dir, "val"), exist_ok=True)
        
        print(f"数据收集器初始化完成，数据将保存到: {save_dir}")

        # 改为双缓冲设计（带时间戳）
        self.lock = threading.Lock()
        self.image_buffer = collections.deque(maxlen=1) # (timestamp, left_img, right_img， head_img)
        self.joint_buffer = collections.deque(maxlen=1) # (timestamp, joint_pos)

        self.lock = threading.Lock()
        self.latest_state_q = np.zeros(14, np.float32)
        self.latest_head_image = None
        self.latest_l_wrist_image = None
        self.latest_r_wrist_image = None

    def update_images(self, left_img, right_img, head_img):
        timestamp = time.perf_counter()
        with self.lock:
            self.image_buffer.append((timestamp, left_img, right_img, head_img))
            # self.image_buffer.append((timestamp, left_img, right_img, None))

    def update_joints_state(self, joint_pos):
        timestamp = time.perf_counter()
        with self.lock:
            self.joint_buffer.append((timestamp, joint_pos.copy())) # copy防止内存被复用出现部分覆盖,回调函数容易复用内存

    def add_step(self):
        """添加一个时间步的数据"""
        with self.lock:
            img_data = self.image_buffer[-1] if self.image_buffer else (None, None, None, None)
            joint_data = self.joint_buffer[-1] if self.joint_buffer else (None, None)
        time_diff = abs(img_data[0] - joint_data[0]) * 1000 if img_data[0] and joint_data[0] else 0

        if joint_data[0] and img_data[0]:
            joint_pos = joint_data[1]
            eef_pos = self.kinematics_solver.compute_arm_poses(joint_pos)
            
            # 创建数据点
            step_data = {
                'image': img_data[3],  # 使用左相机作为主图像
                'l_wrist_image': img_data[1],  # 使用左相机作为左手腕图像
                'r_wrist_image': img_data[2],  # 使用右相机作为右手腕图像
                'joint_pos': np.array(joint_pos, dtype=np.float32),
                'eef_pos': np.array(eef_pos, dtype=np.float32),
                'action': np.zeros(14, dtype=np.float32),  # 占位符动作
                'language_instruction': 'robot operation',
            }
            
            self.current_episode.append(step_data)
            self.step_count += 1
            
            # 当达到episode长度时，换下一个episode保存
            if self.step_count >= self.episode_length:
                self.episodes.append(self.current_episode)
                self.episode_count += 1

                self.current_episode = []
                self.step_count = 0

    def save_episode(self):
        """保存完整的episodes 为NumPy文件"""
        for i, episode in enumerate(self.episodes):
            filename = f"episode_{i}.npy"
            filepath = os.path.join(self.save_dir, "train", filename)
            np.save(filepath, np.array(episode), allow_pickle=True)
            print(f"已保存episode {i} 到 {filepath}")
        
class ArmKinematicsSolver:
    def __init__(self, urdf_path):
        # 加载URDF模型
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        
        # 获取手臂末端执行器帧ID
        self.left_ee_id = self.model.getFrameId("Left_Hand_Roll")
        self.right_ee_id = self.model.getFrameId("Right_Hand_Roll")
        
        # 预分配内存
        self.q = pin.neutral(self.model)
        self.left_ee_pose = pin.SE3.Identity()
        self.right_ee_pose = pin.SE3.Identity()
    
    def compute_arm_poses(self, joint_positions):
        for i, pos in enumerate(joint_positions):
            self.q[i] = pos
        
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
        
        # 组合末端执行器位置数据 (3位置 + 4四元数) * 2
        eef_positions = np.concatenate([
            left_pos, left_rot, 
            right_pos, right_rot
        ])
        
        return eef_positions

class DualCameraSubscriber(Node):
    def __init__(self, data_collector, cv_show=False):
        super().__init__('synch_camera_subscriber')
        self.data_collector = data_collector
        
        self.message_count = 0
        self.start_time = time.time()
        self.cv_render_flag = cv_show
        self.last_frame_time = time.time()  # 上一帧时间

        # 创建相机订阅器
        self.head_sub = Subscriber(self, Image, '/camera/camera/color/image_raw')
        self.left_sub = Subscriber(self, Image, '/camera_left/color/image_raw')
        self.right_sub = Subscriber(self, Image, '/camera_right/color/image_raw')
        
        # 创建时间同步器
        self.sync = ApproximateTimeSynchronizer(
            [self.head_sub, self.left_sub, self.right_sub],
            # [self.left_sub, self.right_sub],
            queue_size=20,
            slop=0.1
        )
        self.sync.registerCallback(self.sync_callback)
        
        if self.cv_render_flag:
            cv2.namedWindow("Synchronized Stereo Cameras", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Synchronized Stereo Cameras", 1200, 400)
        
        self.max_diff_history = []
        self.avg_delay_history = []

    def manual_image_conversion(self, msg):
        try:
            if msg.encoding == 'bgr8':
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                rgb_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    msg.height, msg.width, 3)
                cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().error(f"不支持的编码格式: {msg.encoding}")
                return None
            return cv_image
        except Exception as e:
            self.get_logger().error(f"手动图像转换失败: {str(e)}")
            return None
        
    def sync_callback(self, head_msg, left_msg, right_msg):       
    # def sync_callback(self, left_msg, right_msg):
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
            self.get_logger().info(f"收到头 图像: 宽度={head_msg.width}, 高度={head_msg.height}, 编码={head_msg.encoding}")
            self.get_logger().info(f"收到左 图像: 宽度={left_msg.width}, 高度={left_msg.height}")
            self.get_logger().info(f"收到右 图像: 宽度={right_msg.width}, 高度={right_msg.height}")

        left_img = self.manual_image_conversion(left_msg)
        right_img = self.manual_image_conversion(right_msg)
        head_img = self.manual_image_conversion(head_msg)
        
        # 存储图像数据（状态数据将在状态处理器中添加）
        self.data_collector.update_images(left_img, right_img, head_img)
        # self.data_collector.update_images(left_img, right_img)

        if self.cv_render_flag and left_img is not None and right_img is not None:
            try:
                # 调整图像大小
                def resize_img(img, max_height=400):
                    h, w = img.shape[:2]
                    scale = max_height / h
                    return cv2.resize(img, (int(w * scale), int(h * scale)))
                
                left_img = resize_img(left_img)
                right_img = resize_img(right_img)
                
                # 创建并排视图
                combined = np.hstack((left_img, right_img))
                
                # 添加信息
                cv2.putText(combined, "Left Camera", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(combined, "Right Camera", (left_img.shape[1] + 10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.line(combined, (left_img.shape[1], 0), 
                        (left_img.shape[1], left_img.shape[0]), (0, 255, 0), 2)
                
                # 显示时间同步信息
                left_timestamp = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec/1e9
                right_timestamp = right_msg.header.stamp.sec + right_msg.header.stamp.nanosec/1e9
                time_diff = abs(left_timestamp - right_timestamp)
                sync_info = f"Frame #{self.message_count} | Image Diff: {time_diff*1000:.1f}ms"
                cv2.putText(combined, sync_info, (10, combined.shape[0] - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # 显示图像
                cv2.imshow("Synchronized Stereo Cameras", combined)
                cv2.waitKey(1)
                
            except Exception as e:
                self.get_logger().error(f"图像处理错误: {str(e)}")

    def __del__(self):
        cv2.destroyAllWindows()

class B1DataProcessor:
    def __init__(self, data_collector):
        self.data_collector = data_collector
        self.joint_q = np.zeros(14 , dtype=np.float32)

    def _state_feedback_handler(self, low_state_msg):
        for i, motor in enumerate(low_state_msg.motor_state_serial[2:16]):
            self.joint_q[i] = motor.q
        # print(f"  关节位置: {np.array(self.joint_q)}")
        self.data_collector.update_joints_state(self.joint_q)

def run_ros_node(data_collector):
    rclpy.init()
    node = DualCameraSubscriber(data_collector)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("键盘中断，关闭节点...")
    except Exception as e:  
        node.get_logger().error(f"节点运行时出错: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main():
    
    # 创建数据收集器
    data_collector = DataCollector(save_dir="rlds_data", episode_length=100)
    
    # 启动ROS相机订阅线程（30Hz）
    ros_thread = threading.Thread(target=run_ros_node, args=(data_collector,))
    ros_thread.daemon = True
    ros_thread.start()

    # 启动关节状态订阅线程 (500Hz)
    ChannelFactory.Instance().Init(0)
    processor = B1DataProcessor(data_collector)
    channel_subscriber = B1LowStateSubscriber(processor._state_feedback_handler)
    channel_subscriber.InitChannel()
    
    ctl_T = 1 / 50  # 50Hz的存储数据
    try:
        # 主线程保持运行，等待退出信号
        while data_collector.running:
            start = time.time()

            data_collector.add_step()

            elapsed_time = time.time() - start
            print(f"耗时： {elapsed_time}")
            sleep_time = ctl_T - elapsed_time
            if sleep_time < 0:
                print(f"数据记录超时， 退出程序")
                data_collector.running = False
                break
            time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭程序...")
        # 确保保存episodes
        print(f"\n episode 个数： {data_collector.episode_count}")
        data_collector.save_episode()

if __name__ == "__main__":
    main()