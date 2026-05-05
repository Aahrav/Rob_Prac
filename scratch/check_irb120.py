
import numpy as np
from typing import List, Optional

class DHJoint:
    def __init__(self, joint_type: str, theta: float, d: float, a: float, alpha: float, name: str = ""):
        self.type = joint_type
        self.theta = float(theta)
        self.d = float(d)
        self.a = float(a)
        self.alpha = float(alpha)
        self.name = name

class KinematicChain:
    def __init__(self, joints: List[DHJoint], base_height: float = 0.0):
        self.joints = joints
        self.base_height = base_height

    def forward_kinematics(self, joint_angles: List[float] = None) -> List[np.ndarray]:
        n = len(self.joints)
        if joint_angles is None:
            joint_angles = [0.0] * n
        
        transforms = [np.eye(4)]
        transforms[0][2, 3] = self.base_height
        
        for i, joint in enumerate(self.joints):
            q_val = joint_angles[i]
            if joint.type == 'revolute':
                theta = np.radians(q_val)
                d = joint.d
            elif joint.type == 'prismatic':
                theta = np.radians(joint.theta)
                d = q_val
            else:
                theta = np.radians(joint.theta)
                d = joint.d
            
            a = joint.a
            alpha = np.radians(joint.alpha)
            
            cth, sth = np.cos(theta), np.sin(theta)
            cal, sal = np.cos(alpha), np.sin(alpha)
            T = np.array([
                [cth, -sth * cal,  sth * sal, a * cth],
                [sth,  cth * cal, -cth * sal, a * sth],
                [0,    sal,        cal,       d],
                [0,    0,          0,         1]
            ])
            transforms.append(transforms[-1] @ T)
        return transforms

def preset_ur5():
    joints = [
        DHJoint('revolute', 0,  0.0892,  0.0,     90.0,  "J1"),
        DHJoint('revolute', 0,  0.0,    -0.4250,   0.0,  "J2"),
        DHJoint('revolute', 0,  0.0,    -0.3922,   0.0,  "J3"),
        DHJoint('revolute', 0,  0.1093,  0.0,      90.0, "J4"),
        DHJoint('revolute', 0,  0.0948,  0.0,     -90.0, "J5"),
        DHJoint('revolute', 0,  0.0825,  0.0,       0.0, "J6"),
    ]
    return KinematicChain(joints)

def test_angles(angles):
    ee = chain.forward_kinematics(angles)[-1][:3, 3]
    print(f"Angles {angles} -> EE: {ee}")

chain = preset_ur5()
test_angles([0, 0, 0, 0, 0, 0])
test_angles([0, -90, 0, 0, 0, 0])
