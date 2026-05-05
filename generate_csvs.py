import sys
import os
import csv
from pathlib import Path

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend.kinematics import preset_3dof_arm
from backend.robot_presets import PRESETS

def generate_csv_for_chain(name, chain, joint_configs, out_dir):
    filename = name.replace(" ", "_").replace("/", "_").replace("-", "_") + ".csv"
    filepath = out_dir / filename
    
    with open(filepath, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z'])
        
        for angles in joint_configs:
            ee_pos = chain.forward_kinematics(angles)[-1][:3, 3]
            writer.writerow([f"{ee_pos[0]:.3f}", f"{ee_pos[1]:.3f}", f"{ee_pos[2]:.3f}"])
            
    print(f"Generated {filepath}")

def main():
    out_dir = Path(os.path.dirname(os.path.abspath(__file__))) / "trajectories"
    out_dir.mkdir(exist_ok=True)
    
    # 3-DOF
    generate_csv_for_chain("Standard 3-DOF", preset_3dof_arm(), [
        [0, 30, -30], [45, 45, -45], [-45, 60, -90]
    ], out_dir)
    
    # SCARA
    generate_csv_for_chain("SCARA", PRESETS["SCARA"]["factory"](), [
        [0, 0, 0, 0.05], [45, -45, 0, 0.10], [-30, 60, 0, 0.0]
    ], out_dir)
    
    # UR5
    generate_csv_for_chain("UR5", PRESETS["UR5 (6-DOF)"]["factory"](), [
        [0, -90, 0, -90, 0, 0], [45, -45, 90, -45, 90, 0], [-45, -120, -60, -90, -90, 0]
    ], out_dir)
    
    # PUMA
    generate_csv_for_chain("PUMA 560", PRESETS["PUMA 560"]["factory"](), [
        [0, 0, 0, 0, 0, 0], [45, 45, -45, 0, 45, 0], [-30, 90, -90, 0, -45, 0]
    ], out_dir)
    
    # Stanford
    generate_csv_for_chain("Stanford Arm", PRESETS["Stanford Arm"]["factory"](), [
        [0, 45, 0.3, 0, 0, 0], [45, 0, 0.5, 0, 45, 0], [-45, 60, 0.2, 0, -45, 0]
    ], out_dir)
    
    # ABB
    generate_csv_for_chain("ABB IRB 120", PRESETS["ABB IRB 120"]["factory"](), [
        [0, 0, 0, 0, 0, 0], [45, 45, -30, 0, 45, 0], [-45, -30, 30, 0, -45, 0]
    ], out_dir)

if __name__ == "__main__":
    main()
