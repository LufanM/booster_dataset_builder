#!/usr/bin/env python3

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sqlite3
import numpy as np
import cv2
import yaml
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
import time
import matplotlib
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from booster_dataset.utils.compute_fk import ArmKinematicsSolver

# 在服务器环境中使用非交互式后端
matplotlib.use('TkAgg')  # 使用非交互式后端
import matplotlib.pyplot as plt

class BagToNpyConverter:
    def __init__(self, bag_path, output_dir, r_type, data_visualization=True):
        self.bag_path = bag_path
        self.output_dir = output_dir
        self.data_fps = 30  # 数据采样频率为30Hz
        self.r_type = r_type  # 记录类型："1": 三相机数据和joint postion；"2": 头相机和joint postion
        os.makedirs(output_dir, exist_ok=True)
        
        # 读取rosbag2的文件
        metadata_path = os.path.join(bag_path, 'metadata.yaml')
        with open(metadata_path, 'r') as f:
            self.metadata = yaml.safe_load(f)
        
        db3_path = os.path.join(bag_path, self.metadata['rosbag2_bagfile_information']['relative_file_paths'][0])
        self.conn = sqlite3.connect(db3_path)
        self.cursor = self.conn.cursor()
        
        # 获取消息类型
        self.msg_types = {}
        for topic in self.metadata['rosbag2_bagfile_information']['topics_with_message_count']:
            topic_name = topic['topic_metadata']['name']
            msg_type = topic['topic_metadata']['type']
            self.msg_types[topic_name] = get_message(msg_type)
        
        print(f"Found topics: {list(self.msg_types.keys())}")
    
        self.data_visualization = data_visualization  # 是否可视化图像

        self.kinematics_solver = ArmKinematicsSolver("./../booster_dataset/utils/T1_7DofArm_Serial.urdf")

    def extract_messages(self, topic_name):
        """从数据库中提取指定主题的所有消息"""
        # 首先查询主题ID
        topic_id_query = "SELECT id FROM topics WHERE name = ?"
        self.cursor.execute(topic_id_query, (topic_name,))
        topic_id_result = self.cursor.fetchone()
        
        if topic_id_result is None:
            print(f"Warning: Topic '{topic_name}' not found in database")
            return []
        
        topic_id = topic_id_result[0]
        
        # 使用主题ID查询消息
        query = "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp"
        self.cursor.execute(query, (topic_id,))
        rows = self.cursor.fetchall()
        
        messages = []
        for row in rows:
            timestamp, data = row
            try:
                msg_type = self.msg_types[topic_name]
                msg = deserialize_message(data, msg_type)
                messages.append((timestamp, msg))
            except Exception as e:
                print(f"Error deserializing message from {topic_name}: {e}")
        
        print(f"Extracted {len(messages)} messages from {topic_name}")
        return messages
    
    def image_msg_to_numpy(self, img_msg):
        """将ROS2图像消息转换为numpy数组"""
        try:
            if img_msg.encoding == 'bgr8':
                cv_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    img_msg.height, img_msg.width, 3)
                rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            elif img_msg.encoding == 'rgb8':
                rgb_image = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(
                    img_msg.height, img_msg.width, 3)
                cv_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
            return rgb_image, cv_image
        except Exception as e:
            print(f"Error converting image: {e}")
            return None, None
        
    def sync_three_cameras(self, head_msgs, left_msgs, right_msgs, max_skew_ns=20_000_000):
        """
        在不重复使用任一相机帧的前提下，对齐三相机时间戳。
        采用以头相机为主的单调前进贪心三向匹配：
        - 对每个 head 帧，向前推进左右指针，寻找各自“最近”的未使用帧
        - 若三者与 head 的偏差均 <= max_skew_ns，则接收该三元组
        - 采用三者中位数作为该同步帧的代表时间（抑制抖动）
        参数:
            max_skew_ns: 允许的最大时间偏差(纳秒)。常用 10~20ms。
        返回:
            synced_triplets: [(t_sync, head_idx, left_idx, right_idx), ...]
        """
        th = np.array([t for t, _ in head_msgs], dtype=np.int64)
        tl = np.array([t for t, _ in left_msgs], dtype=np.int64) if left_msgs else np.array([], dtype=np.int64)
        tr = np.array([t for t, _ in right_msgs], dtype=np.int64) if right_msgs else np.array([], dtype=np.int64)

        i = j = k = 0
        n_h, n_l, n_r = len(th), len(tl), len(tr)
        synced_triplets = []

        while i < n_h and j < n_l and k < n_r:
            t_head = th[i]

            # 把 j,k 往前推到最靠近 t_head 的位置（保持单调，避免重复）
            while j + 1 < n_l and abs(tl[j + 1] - t_head) <= abs(tl[j] - t_head):
                j += 1
            while k + 1 < n_r and abs(tr[k + 1] - t_head) <= abs(tr[k] - t_head):
                k += 1

            dl = abs(tl[j] - t_head)
            dr = abs(tr[k] - t_head)

            if dl <= max_skew_ns and dr <= max_skew_ns:
                t_sync = int(np.median([t_head, tl[j], tr[k]]))
                synced_triplets.append((t_sync, i, j, k))
                # 接受该三元组后，三路都前进，保证“不重复使用帧”
                i += 1
                j += 1
                k += 1
            else:
                # 谁更“落后”就推进谁（让三者往一起靠）
                # 若两侧都远，则优先推进时间更早的一侧
                t_min = min(t_head, tl[j] if j < n_l else np.iinfo(np.int64).max,
                            tr[k] if k < n_r else np.iinfo(np.int64).max)
                if t_min == t_head:
                    i += 1
                elif t_min == tl[j]:
                    j += 1
                else:
                    k += 1

        return synced_triplets
    
    def visualize_three_cameras(self, head_img, left_img, right_img, t_sync, frame_idx, total_frames):
        """可视化三个相机的同步图像"""
        if head_img is None or left_img is None or right_img is None:
            return
        
        if frame_idx == 0:
            # 获取目标高度
            target_height = 240
            
            # 调整所有图像到相同高度，保持宽高比
            def resize_to_height(img, target_height):
                h, w = img.shape[:2]
                ratio = target_height / h
                new_width = int(w * ratio)
                return cv2.resize(img, (new_width, target_height))
            
            # 调整所有图像尺寸
            head_img_resized = resize_to_height(head_img, target_height)
            left_img_resized = resize_to_height(left_img, target_height)
            right_img_resized = resize_to_height(right_img, target_height)
            
            # 水平拼接三个图像
            combined_img = np.hstack((head_img_resized, left_img_resized, right_img_resized))
            
            # 获取视频尺寸
            height, width = combined_img.shape[:2]
            
            # 创建输出目录（如果不存在）
            os.makedirs("visualization_videos", exist_ok=True)
            
            # 创建视频写入器
            fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 编码格式
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = f"visualization_videos/three_cameras_{timestamp}.avi"
            self.video_writer = cv2.VideoWriter(output_path, fourcc, self.data_fps, (width, height))
            print(f"视频将保存到: {output_path}")

        # 获取目标高度（选择三个图像中最常见的高度）
        target_height = 240  # 根据错误信息，left_img的高度是240
        
        # 调整所有图像到相同高度，保持宽高比
        def resize_to_height(img, target_height):
            h, w = img.shape[:2]
            ratio = target_height / h
            new_width = int(w * ratio)
            return cv2.resize(img, (new_width, target_height))
        
        # 调整所有图像尺寸
        head_img = resize_to_height(head_img, target_height)
        left_img = resize_to_height(left_img, target_height)
        right_img = resize_to_height(right_img, target_height)
        
        # 水平拼接三个图像
        combined_img = np.hstack((head_img, left_img, right_img))
        
        # 写入视频帧
        self.video_writer.write(combined_img)

        # 添加文本信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_color = (255, 255, 255)
        thickness = 2
        
        # 获取各个图像的宽度用于文本定位
        w_head = head_img.shape[1]
        w_left = left_img.shape[1]
        
        # 添加相机标签
        cv2.putText(combined_img, "Head Camera", (10, 30), font, font_scale, font_color, thickness)
        cv2.putText(combined_img, "Left Wrist", (w_head + 10, 30), font, font_scale, font_color, thickness)
        cv2.putText(combined_img, "Right Wrist", (w_head + w_left + 10, 30), font, font_scale, font_color, thickness)
        
        # 添加帧信息和时间戳
        info_text = f"Frame: {frame_idx+1}/{total_frames}, Time: {t_sync/1e9:.3f}s"
        cv2.putText(combined_img, info_text, (10, target_height - 10), font, font_scale, font_color, thickness)
        
        # 显示图像
        cv2.imshow("Three Camera Synchronization", combined_img)

        delay = max(1, int(1000 / self.data_fps))  # 确保至少1ms延迟
        key = cv2.waitKey(delay) & 0xFF
        # 等待按键（1ms），按'q'退出
        if key == ord('q'):
            return False
        elif key == ord(' '):  # 空格键暂停/继续
            while True:
                key2 = cv2.waitKey(0) & 0xFF
                if key2 == ord(' '):  # 再次按空格继续
                    break
                elif key2 == ord('q'):  # 暂停状态下按q退出
                    return False
        
        return True
    
    def visualize_head_cameras(self, head_img, frame_idx, time_stamp_i, total_frames):
        """可视化三个相机的同步图像"""
        if head_img is None:
            return
        
        if frame_idx == 0:
            # 获取目标高度
            target_height = 240
            
            # 调整所有图像到相同高度，保持宽高比
            def resize_to_height(img, target_height):
                h, w = img.shape[:2]
                ratio = target_height / h
                new_width = int(w * ratio)
                return cv2.resize(img, (new_width, target_height))
            
            # 调整所有图像尺寸
            head_img_resized = resize_to_height(head_img, target_height)
            
            # 水平拼接三个图像
            combined_img = np.hstack((head_img_resized))
            height, width = combined_img.shape[:2]
            
            # 创建输出目录（如果不存在）
            os.makedirs("visualization_videos", exist_ok=True)
            
            # 创建视频写入器
            fourcc = cv2.VideoWriter_fourcc(*'XVID')  # 编码格式
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_path = f"visualization_videos/three_cameras_{timestamp}.avi"
            self.video_writer = cv2.VideoWriter(output_path, fourcc, self.data_fps, (width, height))
            print(f"视频将保存到: {output_path}")

        # 获取目标高度（选择三个图像中最常见的高度）
        target_height = 240  
        
        # 调整所有图像到相同高度，保持宽高比
        def resize_to_height(img, target_height):
            h, w = img.shape[:2]
            ratio = target_height / h
            new_width = int(w * ratio)
            return cv2.resize(img, (new_width, target_height))
        
        # 调整所有图像尺寸
        head_img = resize_to_height(head_img, target_height)
        combined_img = np.hstack((head_img))
        
        # 写入视频帧
        self.video_writer.write(combined_img)

        # 添加文本信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        font_color = (255, 255, 255)
        thickness = 2
        
        # 获取各个图像的宽度用于文本定位
        w_head = head_img.shape[1]
        
        # 添加相机标签
        cv2.putText(combined_img, "Head Camera", (10, 30), font, font_scale, font_color, thickness)
        
        # 添加帧信息和时间戳
        info_text = f"Frame: {frame_idx+1}/{total_frames}, Time: {time_stamp_i/1e9:.3f}s"
        cv2.putText(combined_img, info_text, (10, target_height - 10), font, font_scale, font_color, thickness)
        
        # 显示图像
        cv2.imshow("Three Camera Synchronization", combined_img)

        delay = max(1, int(1000 / self.data_fps))  # 确保至少1ms延迟
        key = cv2.waitKey(delay) & 0xFF
        # 等待按键（1ms），按'q'退出
        if key == ord('q'):
            return False
        elif key == ord(' '):  # 空格键暂停/继续
            while True:
                key2 = cv2.waitKey(0) & 0xFF
                if key2 == ord(' '):  # 再次按空格继续
                    break
                elif key2 == ord('q'):  # 暂停状态下按q退出
                    return False
        
        return True
       
    def plot_interpolation_comparison(self, low_state_timestamps, joint_positions, matched_timestamps, joint_interp):
        """绘制插值前后的关节数据对比图并显示"""
        try:
            # 选择前几个关节进行可视化
            num_joints_to_plot = min(6, joint_positions.shape[1])
            
            fig, axes = plt.subplots(num_joints_to_plot, 1, figsize=(12, 3*num_joints_to_plot))
            if num_joints_to_plot == 1:
                axes = [axes]
            
            # 转换时间戳为相对时间（秒）
            base_time = low_state_timestamps[0]
            low_state_times_sec = [(t - base_time) / 1e9 for t in low_state_timestamps]
            matched_times_sec = [(t - base_time) / 1e9 for t in matched_timestamps]
            
            for joint_idx in range(num_joints_to_plot):
                ax = axes[joint_idx]
                
                # 绘制原始数据点
                ax.plot(low_state_times_sec, joint_positions[:, joint_idx], 
                       'bo', markersize=1.2, alpha=0.7, label='raw_data')
                
                # 绘制插值后的数据点
                interp_values = [joint_interp(t)[joint_idx] for t in matched_timestamps]
                ax.plot(matched_times_sec, interp_values, 
                       'ro', markersize=1.2, label='interpolated_data')
                
                # # 对于线性插值，连接原始数据点以显示插值的基础
                # ax.plot(low_state_times_sec, joint_positions[:, joint_idx], 
                #     'g-', alpha=0.3)
                
                ax.set_ylabel(f'Joint {joint_idx+1} position (rad)')
                ax.grid(True, alpha=0.3)
                ax.legend()
                
                if joint_idx == num_joints_to_plot - 1:
                    ax.set_xlabel('time (s)')

            plt.suptitle('joint position interpolation comparison')
            plt.tight_layout()
            plt.savefig(os.path.join("visualization_videos", 'joint_interpolation_comparison.png'), dpi=300)
            plt.show()
            plt.close()
            print("插值对比图已显示，关闭窗口以继续...")
        except Exception as e:
            print(f"无法生成图表: {e}")

    def process_bag(self):
        """处理整个bag文件"""
        # 提取所有需要的消息
        if self.r_type == "1":
            topics_to_extract = [
                '/camera/color/image_raw',
                '/camera_left/color/image_raw', 
                '/camera_right/color/image_raw',
                '/low_state'
            ]
            
            extracted_data = {}
            for topic in topics_to_extract:
                if topic in self.msg_types:
                    extracted_data[topic] = self.extract_messages(topic)
                else:
                    print(f"Warning: Topic {topic} not found in metadata")
                    extracted_data[topic] = []
            
            head_camera_msgs = extracted_data['/camera/color/image_raw']
            left_camera_msgs = extracted_data['/camera_left/color/image_raw']
            right_camera_msgs = extracted_data['/camera_right/color/image_raw']
            low_state_msgs = extracted_data['/low_state']

            # 检查是否有足够的数据
            if not head_camera_msgs:
                print("Error: No head camera messages found")
                return
            if not left_camera_msgs:
                print("Error: No left camera messages found")
                return
            if not right_camera_msgs:
                print("Error: No right camera messages found")
                return
            if not low_state_msgs:
                print("Error: No low state messages found")
                return
            
            # 1. 找到三个相机时间戳最对齐的三元组
            matches = self.sync_three_cameras(
                head_camera_msgs, left_camera_msgs, right_camera_msgs, max_skew_ns=30_000_000
            )
            
            if not matches:
                print("No valid triplet matches found!")
                return
            
            # 2. 从LowState消息中提取关节位置
            low_state_timestamps = [msg[0] for msg in low_state_msgs]
            joint_positions = []
            
            for _, msg in low_state_msgs:
                if hasattr(msg, 'motor_state_serial') and len(msg.motor_state_serial) > 0:
                    joint_pos = [motor.q for motor in msg.motor_state_serial]
                    joint_positions.append(joint_pos)
                else:
                    joint_positions.append([])
                    
            
            # 3. 创建插值函数用于关节数据
            joint_positions = np.array(joint_positions, dtype=np.float32)
            if len(low_state_timestamps) > 1 and len(joint_positions) > 0:
                joint_interp = interp1d(
                    low_state_timestamps, 
                    joint_positions, 
                    axis=0,
                    kind='linear',  # 使用线性插值， 5次多项式插值 cubic样条插值会有过冲
                    bounds_error=False,  # 不允许外推
                    fill_value='extrapolate'  # 但实际上我们已经确保不会外推
                )
            else:
                print("Not enough data for interpolation")
                return
            
            # 检查同步时间是否在 LowState 时间范围内
            t_sync_start = matches[0][0]
            t_sync_end = matches[-1][0]
            if t_sync_start < low_state_timestamps[0] or t_sync_end > low_state_timestamps[-1]:
                print(f"警告: 同步时间 {t_sync} 超出 LowState 时间范围 [{low_state_timestamps[0]}, {low_state_timestamps[-1]}]")
                return

            # 4. 处理每个匹配的三元组
            all_step_data = []
            print("Processing matched triplets...")
            output_path = os.path.join(self.output_dir, 'episode_teleop.npy')

            print(f"head Image的分辨为: height*width = {head_camera_msgs[0][1].height}*{head_camera_msgs[0][1].width}\n",
                f"left Image的分辨为: height*width = {left_camera_msgs[0][1].height}*{left_camera_msgs[0][1].width}\n",
                f"right Image的分辨为: height*width = {right_camera_msgs[0][1].height}*{right_camera_msgs[0][1].width}\n")
            
            for i, (t_sync, head_idx, left_idx, right_idx) in enumerate(matches):
                # 获取图像
                head_img, cv_head_img = self.image_msg_to_numpy(head_camera_msgs[head_idx][1])
                left_img, cv_left_img = self.image_msg_to_numpy(left_camera_msgs[left_idx][1])
                right_img, cv_right_img = self.image_msg_to_numpy(right_camera_msgs[right_idx][1])
                
                if head_img is None or left_img is None or right_img is None:
                    continue
                
                # 插值获取关节位置和末端执行器位置
                joint_pos = joint_interp(t_sync)
                eef_pos = self.kinematics_solver.compute_arm_eef(joint_pos)
                
                step_data = {
                    'image': head_img,
                    'l_wrist_image': left_img,
                    'r_wrist_image': right_img,
                    'joint_pos': np.array(joint_pos[2:16], dtype=np.float32), # 只保留手臂相关的14个关节
                    'eef_pos': np.array(eef_pos, dtype=np.float32),
                    'action': np.zeros(14, dtype=np.float32),
                    'language_instruction': 'robot teleoperation',
                }
                
                all_step_data.append(step_data)
                
                if (i+1) % 100 == 0 and i > 0:
                    filename = f"episode_teleop_step_{i//100}.npy"
                    output_path = os.path.join(self.output_dir, filename)
                    np.save(output_path, all_step_data)        
                    print(f"Processed {i+1}/{len(matches)} triplets, Saved {len(all_step_data)} synchronized frames to {output_path}" )
                    all_step_data = []
                elif i == len(matches) - 1:
                    filename = f"episode_teleop_step_{(i//100)}.npy"
                    output_path = os.path.join(self.output_dir, filename)
                    np.save(output_path, all_step_data)        
                    print(f"Processed {i+1}/{len(matches)} triplets, Saved {len(all_step_data)} synchronized frames to {output_path}" )
                
                # 相机图像可视化
                if self.data_visualization:
                    continue_rendering = self.visualize_three_cameras(
                        cv_head_img, cv_left_img, cv_right_img, t_sync, i, len(matches)
                    )
                    if not continue_rendering:
                        print("用户中断了可视化")
                        break

            # 5. 绘制并显示插值对比图
            if self.data_visualization:
                matched_timestamps = [m[0] for m in matches]
                self.plot_interpolation_comparison(
                    low_state_timestamps, 
                    joint_positions, 
                    matched_timestamps, 
                    joint_interp
                )              
        # 如果只记录头相机和关节位置
        else:
            # 1. 解析头相机和low_state数据
            topics_to_extract = [
                '/camera/color/image_raw',
                '/low_state'
            ]
            
            extracted_data = {}
            for topic in topics_to_extract:
                if topic in self.msg_types:
                    extracted_data[topic] = self.extract_messages(topic)
                else:
                    print(f"Warning: Topic {topic} not found in metadata")
                    extracted_data[topic] = []
            
            head_camera_msgs = extracted_data['/camera/color/image_raw']
            low_state_msgs = extracted_data['/low_state']

            # 检查是否有足够的数据
            if not head_camera_msgs:
                print("Error: No head camera messages found")
                return
            if not low_state_msgs:
                print("Error: No low state messages found")
                return
            
            # 2. 从LowState消息中提取关节位置
            low_state_timestamps = [msg[0] for msg in low_state_msgs]
            joint_positions = []
            
            for _, msg in low_state_msgs:
                if hasattr(msg, 'motor_state_serial') and len(msg.motor_state_serial) > 0:
                    joint_pos = [motor.q for motor in msg.motor_state_serial]
                    joint_positions.append(joint_pos)
                else:
                    joint_positions.append([])

            # 3. 处理数据生成step_data
            all_step_data = []
            output_path = os.path.join(self.output_dir, 'episode_teleop.npy')

            print(f"head Image的分辨为: height*width = {head_camera_msgs[0][1].height}*{head_camera_msgs[0][1].width}\n")
            
            for i, _ in enumerate(head_camera_msgs):
                # 获取图像
                head_img, cv_head_img = self.image_msg_to_numpy(head_camera_msgs[i][1])
                
                joint_pos = joint_positions[i]
                eef_pos = self.kinematics_solver.compute_arm_eef(joint_pos)
                
                step_data = {
                    'image': head_img,
                    'l_wrist_image': None,
                    'r_wrist_image': None,
                    'joint_pos': np.array(joint_pos[2:16], dtype=np.float32), # 只保留手臂相关的14个关节
                    'eef_pos': np.array(eef_pos, dtype=np.float32),
                    'action': np.zeros(14, dtype=np.float32),
                    'language_instruction': 'robot teleoperation',
                }
                
                all_step_data.append(step_data)
                
                if (i+1) % 100 == 0 and i > 0:
                    filename = f"episode_teleop_step_{i//100}.npy"
                    output_path = os.path.join(self.output_dir, filename)
                    np.save(output_path, all_step_data)        
                    print(f"Processed {i+1}/{len(head_camera_msgs)} triplets, Saved {len(all_step_data)} synchronized frames to {output_path}" )
                    all_step_data = []
                elif i == len(head_camera_msgs) - 1:
                    filename = f"episode_teleop_step_{(i//100)}.npy"
                    output_path = os.path.join(self.output_dir, filename)
                    np.save(output_path, all_step_data)        
                    print(f"Processed {i+1}/{len(head_camera_msgs)} triplets, Saved {len(all_step_data)} synchronized frames to {output_path}" )
                
                time_stamp_i = head_camera_msgs[i][0]
                # 相机图像可视化
                if self.data_visualization:
                    continue_rendering = self.visualize_head_cameras(
                        cv_head_img, i, time_stamp_i,len(head_camera_msgs)
                    )
                    if not continue_rendering:
                        print("用户中断了可视化")
                        break

    def close(self):
        """关闭数据库连接"""
        self.conn.close()

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Convert ROS2 bag to NPY format')
    parser.add_argument('--bag_path', help='Path to the ROS2 bag directory')
    parser.add_argument('--output_dir', default="./data", help='Output directory for NPY files')
    parser.add_argument("--r_type", choices=["1", "2"], default="1", help="value '1': three cameras data collection; value '2': head camera data collection")
    
    args = parser.parse_args()
    
    print(f"Converting bag at {args.bag_path} to NPY format in {args.output_dir}")
    # 初始化ROS2（用于消息反序列化）
    rclpy.init()
    
    converter = BagToNpyConverter(args.bag_path, args.output_dir, args.r_type,True)
    
    try:
        converter.process_bag()
    finally:
        converter.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()