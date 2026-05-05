import numpy as np
from backend.robot_presets import PRESETS

for name, meta in PRESETS.items():
    chain = meta["factory"]()
    # Find mid-point for each joint
    mid_angles = []
    max_angles = []
    min_angles = []
    for j in chain.joints:
        lo = j.q_min if j.q_min is not None else -45
        hi = j.q_max if j.q_max is not None else 45
        
        mid = (lo + hi) / 2.0
        # If it's revolute and min/max are wide, just use a reasonable angle like 30 deg
        if j.type == 'revolute' and hi - lo > 180:
            mid = 30.0
            hi_val = 60.0
            lo_val = -30.0
        else:
            hi_val = hi
            lo_val = lo
            
        mid_angles.append(mid)
        max_angles.append(hi_val)
        min_angles.append(lo_val)
        
    pos_mid = chain.forward_kinematics(mid_angles)[-1][:3, 3]
    pos_max = chain.forward_kinematics(max_angles)[-1][:3, 3]
    pos_zero = chain.forward_kinematics([0.0]*len(chain.joints))[-1][:3, 3]
    
    print(f"[{name}]")
    print(f"  Zero pos:   ({pos_zero[0]:.3f}, {pos_zero[1]:.3f}, {pos_zero[2]:.3f})")
    print(f"  Sample pos: ({pos_mid[0]:.3f}, {pos_mid[1]:.3f}, {pos_mid[2]:.3f})")
    print(f"  Stretched:  ({pos_max[0]:.3f}, {pos_max[1]:.3f}, {pos_max[2]:.3f})")
    print()

