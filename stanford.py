# Save this as generate_stanford.py and run it:
# python generate_stanford.py

from backend.robot_presets import preset_stanford_arm

chain = preset_stanford_arm()

configs = [
    [0, 0, 0.3, 0, 0, 0],
    [90, 45, 0.4, 0, 90, 0],
    [-45, -30, 0.2, 45, -45, 90],
    [180, 0, 0.6, 0, 0, 0]
]

for i, q in enumerate(configs):
    # The updated IK engine returns 4x4 transformation matrices
    mats = chain.forward_kinematics(q)
    T_ee = mats[-1]
    
    # Extract the Translation Column (X, Y, Z)
    x, y, z = T_ee[0,3], T_ee[1,3], T_ee[2,3]
    
    print(f'Target {i+1}:')
    print(f'  X: {x:.3f} | Y: {y:.3f} | Z: {z:.3f}')
    print(f'  (Joints: Yaw {q[0]}°, Pitch {q[1]}°, Extend {q[2]}m, Roll {q[3]}°, W_Pitch {q[4]}°, W_Yaw {q[5]}°)\n')
