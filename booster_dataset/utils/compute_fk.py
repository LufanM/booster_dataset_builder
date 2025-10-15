
import numpy as np
import pinocchio as pin

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
    
    def compute_arm_eef(self, joint_positions):
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