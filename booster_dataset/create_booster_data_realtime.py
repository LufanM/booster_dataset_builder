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
import signal
import queue
import logging

class DataCollector:
    def __init__(self, data_type="train", save_dir="data", episode_length=100):
        urdf_path = "./T1_7_dof_arm_serial_with_head_arm.urdf"
        self.kinematics_solver = ArmKinematicsSolver(urdf_path)
        # Setup logging
        logging.basicConfig(level=logging.DEBUG)
        self.logger = logging.getLogger(__name__)

        self.save_dir = save_dir
        self.data_type = data_type
        self.episode_length = episode_length
        self.episode_count = 0
        self.step_count = 0
        self.running = True

        self.latest_state_q = np.zeros(14, np.float32)
        self.latest_head_image = None
        self.latest_l_wrist_image = None
        self.latest_r_wrist_image = None

        # 创建保存目录
        os.makedirs(save_dir, exist_ok=True)
        if self.data_type == "train":
            os.makedirs(os.path.join(save_dir, "train"), exist_ok=True)
        else:
            os.makedirs(os.path.join(save_dir, "val"), exist_ok=True)
        self.logger.info(f"数据收集器初始化完成，数据将保存到: {save_dir}")

        # 改为双缓冲设计（带时间戳）
        self.lock = threading.Lock()
        self.image_buffer = collections.deque(maxlen=1) # (timestamp, left_img, right_img， head_img)
        self.joint_buffer = collections.deque(maxlen=1) # (timestamp, joint_pos)
        self.current_episode = []

        # 使用FIFO队列和单独的保存线程
        self.save_queue = queue.Queue()
        self.save_thread = threading.Thread(target=self._save_worker, daemon=True)
        self.save_thread.start()

        # 用于跟踪待处理的保存任务
        self.pending_episodes = 0  
              
    # def update_images(self, left_img, right_img, head_img):
    # def update_images(self, left_img, right_img):
    def update_images(self, head_img):
        timestamp = time.perf_counter()
        with self.lock:
            # self.image_buffer.append((timestamp, left_img, right_img, head_img))
            # self.image_buffer.append((timestamp, left_img, right_img, None))
            self.image_buffer.append((timestamp, None, None, head_img))


    def update_joints_state(self, joint_pos):
        timestamp = time.perf_counter()
        with self.lock:
            self.joint_buffer.append((timestamp, joint_pos.copy())) # copy防止内存被复用出现部分覆盖,回调函数容易复用内存

    def add_step(self):
        """添加一个时间步的数据"""
        with self.lock:
            img_data = self.image_buffer[-1] if self.image_buffer else (None, None, None, None)
            joint_data = self.joint_buffer[-1] if self.joint_buffer else (None, None)
        time_diff = (img_data[0] - joint_data[0]) * 1000 if img_data[0] and joint_data[0] else 0

        if abs(time_diff) > 100:  # 如果时间差超过100ms，则跳过此帧
            self.logger.warning(f"!! 跳过此帧，时间戳差异过大: {time_diff:.1f} ms  图像时间:{img_data[0]}  关节时间{joint_data[0]}!!")
            return

        # self.logger.debug(f"时间戳差异: {time_diff:.1f} ms  图像时间:{img_data[0]}  关节时间{joint_data[0]}")
        
        if joint_data[0] and img_data[0]:
            joint_pos = joint_data[1]
            eef_pos = self.kinematics_solver.compute_arm_poses(joint_pos)
            
            # 创建数据点
            step_data = {
                'image': img_data[3],  # 使用头相机作为主图像
                'l_wrist_image': img_data[1],  # 使用左相机作为左手腕图像
                'r_wrist_image': img_data[2],  # 使用右相机作为右手腕图像
                'joint_pos': np.array(joint_pos, dtype=np.float32),
                'eef_pos': np.array(eef_pos, dtype=np.float32),
                'action': np.zeros(14, dtype=np.float32),  # 占位符动作
                'language_instruction': 'robot teleoperation',
            }
            
            self.current_episode.append(step_data)
            self.step_count += 1
            
           # 当达到episode长度时，异步保存
            if self.step_count >= self.episode_length:
                t_start = time.perf_counter()
                episode_copy = self.current_episode.copy()
                episode_id = self.episode_count
                # 添加到保存队列
                self.save_queue.put((episode_id, episode_copy))
                self.pending_episodes += 1
                t_end = time.perf_counter()
                # self.logger.info(f"已添加episode {episode_id} 到保存队列，待处理: {self.pending_episodes} 用时: {t_end - t_start:.8f} 秒")
                                
                self.current_episode = []
                self.step_count = 0
                self.episode_count += 1

    def _save_worker(self):
        """保存工作线程，持续从队列中取出并保存episode"""
        while self.running or not self.save_queue.empty():
            try:
                episode_id, episode_data = self.save_queue.get(timeout=1.0)

                self._record_single_episode(episode_id, episode_data)

                # 减少待处理计数
                self.pending_episodes -= 1
                self.logger.info(f"已保存episode {episode_id}。    待处理: {self.pending_episodes}\n")
                
                self.save_queue.task_done()
            except queue.Empty:
                # time.sleep(0.1)
                pass
            except Exception as e:
                self.logger.info(f"保存episode时出错: {e}")
                
    def _record_single_episode(self, episode_count, current_episode):
        """保存单个episode到文件"""
        start_time = time.time()
        filename = f"episode_{episode_count}.npy"
        sub_dir = "train" if self.data_type == "train" else "val"
        filepath = os.path.join(self.save_dir, sub_dir, filename)

        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        np.save(filepath, np.array(current_episode))
        end_time = time.time()
        self.logger.info(f"已保存episode {episode_count} 到 {filepath},  用时 {end_time - start_time:.2f} 秒")

    def stop(self):    
        self.logger.info(f"等待剩余 {self.pending_episodes} 个episode保存...")
        self.save_queue.join()  # 等待队列中所有任务完成
        self.logger.info(f"剩余 {self.pending_episodes} 个episode保存完成!!!")

        self.running = False

        # 保存最后一个不完整的episode TODO Need Check
        if self.current_episode:
            self.logger.info(f"保存最后未完成的 episode {self.episode_count}")
            self._record_single_episode(self.current_episode)

        self.logger.info(f"数据收集器已停止，共保存 {self.episode_count + (1 if self.current_episode else 0)} 个 episode")

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
        # self.left_sub = Subscriber(self, Image, '/camera_left/color/image_raw')
        # self.right_sub = Subscriber(self, Image, '/camera_right/color/image_raw')
        
        # 创建时间同步器
        self.sync = ApproximateTimeSynchronizer(
            # [self.head_sub, self.left_sub, self.right_sub],
            # [self.left_sub, self.right_sub],
            [self.head_sub],
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
        
    # def sync_callback(self, head_msg, left_msg, right_msg):       
    # def sync_callback(self, left_msg, right_msg):
    def sync_callback(self, head_msg):
        """处理同步后的相机帧"""
        self.message_count += 1
        current_time = time.time()

        # 计算帧率
        frame_interval = current_time - self.last_frame_time
        current_fps = 1.0 / frame_interval if frame_interval > 0 else 0
        self.last_frame_time = current_time

        # 每30帧打印一次信息
        if self.message_count % 15 == 0:  
            self.get_logger().info(f"\n=== 帧 #{self.message_count} | 帧率: {current_fps:.2f} Hz ===")
            self.get_logger().info(f"收到头 图像: 高度={head_msg.height}, 宽度={head_msg.width},  编码={head_msg.encoding}")
            # self.get_logger().info(f"收到左 图像: 高度={left_msg.height}, 宽度={left_msg.width}")
            # self.get_logger().info(f"收到右 图像: 高度={right_msg.height}, 宽度={right_msg.width}")

        # left_img = self.manual_image_conversion(left_msg)
        # right_img = self.manual_image_conversion(right_msg)
        head_img = self.manual_image_conversion(head_msg)
        
        # 存储图像数据（状态数据将在状态处理器中添加）
        # self.data_collector.update_images(left_img, right_img, head_img)
        # self.data_collector.update_images(left_img, right_img)
        self.data_collector.update_images(head_img)

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
                # left_timestamp = left_msg.header.stamp.sec + left_msg.header.stamp.nanosec/1e9
                # right_timestamp = right_msg.header.stamp.sec + right_msg.header.stamp.nanosec/1e9
                # time_diff = abs(left_timestamp - right_timestamp)
                # sync_info = f"Frame #{self.message_count} | Image Diff: {time_diff*1000:.1f}ms"
                # cv2.putText(combined, sync_info, (10, combined.shape[0] - 20), 
                #         cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
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
        self.data_collector.update_joints_state(self.joint_q)
        print(f"更新关节状态: {self.joint_q}")



# 全局退出标志和事件
exit_flag = False
exit_event = threading.Event()
def run_ros_node(data_collector, exit_event):
    rclpy.init()
    node = DualCameraSubscriber(data_collector)
    try:
        while not exit_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("键盘中断，关闭节点...")
    except Exception as e:  
        node.get_logger().error(f"节点运行时出错: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("ROS节点已关闭")

def signal_handler(sig, frame):
    """处理Ctrl+C信号"""
    global exit_flag
    print('\n收到中断信号，正在关闭程序...')
    exit_flag = True
    exit_event.set()

def main():
    global exit_flag, exit_event
    signal.signal(signal.SIGINT, signal_handler)

    data_collector = DataCollector(data_type="train", save_dir="data", episode_length=15 * 10)
    
    # 启动ROS相机订阅线程（30Hz）
    ros_thread = threading.Thread(target=run_ros_node, args=(data_collector, exit_event))
    ros_thread.daemon = True
    ros_thread.start()

    # 启动关节状态订阅线程 (500Hz)
    ChannelFactory.Instance().Init(0)
    processor = B1DataProcessor(data_collector)
    channel_subscriber = B1LowStateSubscriber(processor._state_feedback_handler)
    channel_subscriber.InitChannel()

    ctl_T = 1 / 15  # 30Hz的存储数据

    try:
        while data_collector.running  and not exit_event.is_set():
            start = time.time()

            data_collector.add_step()

            elapsed_time = time.time() - start
            sleep_time = ctl_T - elapsed_time
            if sleep_time < 0:
                print(f"数据记录超时， 超时：{-sleep_time}:")
                # data_collector.running = False
                # break
            time.sleep(max(0, sleep_time))

        print(f"退出主线程循环!!")
    except KeyboardInterrupt:
        print("\n收到中断信号，正在关闭程序...")
        exit_event.set()
        exit_flag = True
    except Exception as e:
        print(f"主循环发生异常: {e}")
        exit_event.set()
        exit_flag = True
    finally:
        # 设置退出事件，通知所有线程退出 TODO 这里需要改进 把最后的episode保存下来
        exit_event.set()
        exit_flag = True

        # print("开始保存数据...")
        # data_collector.save_episode()
        # print(f"数据保存完成, episode 个数： {data_collector.episode_count} 被保存到 {data_collector.save_dir}")

        # # 关闭关节状态订阅
        # try:
        #     channel_subscriber.CloseChannel()
        #     print("关节状态订阅已关闭")
        # except Exception as e:
        #     print(f"关闭关节状态订阅时出错: {e}")re

        # # 等待ROS线程结束
        # ros_thread.join(timeout=2.0)
        # if ros_thread.is_alive():
        #     print("警告: ROS线程没有正常退出")
        # else:
        #     print("ROS线程已退出")
        # print("程序退出完成")
        os._exit(0)
  

if __name__ == "__main__":
    main()