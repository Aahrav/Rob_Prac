import sys
import os
import numpy as np

# Adjust python path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend.kinematics import preset_3dof_arm
from backend.robot_presets import PRESETS

def print_targets(name, chain, joint_configs):
    print(f"### {name}")
    for i, angles in enumerate(joint_configs):
        # FK returns a list of matrices. The last one is the end effector.
        ee_pos = chain.forward_kinematics(angles)[-1][:3, 3]
        print(f"- **Target {i+1}**: `X: {ee_pos[0]:.3f}, Y: {ee_pos[1]:.3f}, Z: {ee_pos[2]:.3f}`")
    print()

def main():
    print("Here are some guaranteed reachable (X, Y, Z) targets for various presets:\n")

    # Standard 3-DOF / Elbow Manipulator
    # Joints: Base Yaw, Shoulder Pitch, Elbow Pitch
    chain_3dof = preset_3dof_arm()
    configs_3dof = [
        [0, 30, -30],
        [45, 45, -45],
        [-45, 60, -90]
    ]
    print_targets("Standard 3-DOF / Elbow Manipulator", chain_3dof, configs_3dof)

    # SCARA (4-DOF)
    # Joints: Base Yaw, Elbow Yaw, Wrist Yaw, Vertical (Prismatic 0-0.15)
    chain_scara = PRESETS["SCARA"]["factory"]()
    configs_scara = [
        [0, 0, 0, 0.05],
        [45, -45, 0, 0.10],
        [-30, 60, 0, 0.0]
    ]
    print_targets("SCARA (4-DOF)", chain_scara, configs_scara)

    # UR5 (6-DOF)
    chain_ur5 = PRESETS["UR5 (6-DOF)"]["factory"]()
    configs_ur5 = [
        [0, -90, 0, -90, 0, 0],
        [45, -45, 90, -45, 90, 0],
        [-45, -120, -60, -90, -90, 0]
    ]
    print_targets("UR5 (6-DOF)", chain_ur5, configs_ur5)

    # PUMA 560
    chain_puma = PRESETS["PUMA 560"]["factory"]()
    configs_puma = [
        [0, 0, 0, 0, 0, 0],
        [45, 45, -45, 0, 45, 0],
        [-30, 90, -90, 0, -45, 0]
    ]
    print_targets("PUMA 560", chain_puma, configs_puma)

    # Stanford Arm (RRP-RRR)
    chain_stanford = PRESETS["Stanford Arm"]["factory"]()
    configs_stanford = [
        [0, 45, 0.3, 0, 0, 0],
        [45, 0, 0.5, 0, 45, 0],
        [-45, 60, 0.2, 0, -45, 0]
    ]
    print_targets("Stanford Arm", chain_stanford, configs_stanford)

    # ABB IRB 120
    chain_abb = PRESETS["ABB IRB 120"]["factory"]()
    configs_abb = [
        [0, 0, 0, 0, 0, 0],
        [45, 45, -30, 0, 45, 0],
        [-45, -30, 30, 0, -45, 0]
    ]
    print_targets("ABB IRB 120", chain_abb, configs_abb)

if __name__ == "__main__":
    main()
