import numpy as np
from backend.robot_presets import preset_abb_irb120
from backend.kinematics import KinematicChain

chain = preset_abb_irb120()
target = np.array([0.374, 0.0, 0.630])

res = chain.inverse_kinematics(target, tol=0.005)
if res:
    print("SUCCESS", res)
else:
    print("FAILED")
