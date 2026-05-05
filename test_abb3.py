import numpy as np
from backend.robot_presets import preset_abb_irb120

chain = preset_abb_irb120()

target = np.array([0.374, 0.0, 0.630])
print("\nTesting Target 1 with initial_angles = preset thetas")
thetas = [j.theta for j in chain.joints]
res = chain.inverse_kinematics(target, initial_angles=thetas, tol=0.005)
if res:
    print("SUCCESS", res)
else:
    print("FAILED")

target2 = np.array([0.154, -0.154, 0.645])
print("\nTesting Target 2 with initial_angles = preset thetas")
res2 = chain.inverse_kinematics(target2, initial_angles=thetas, tol=0.005)
if res2:
    print("SUCCESS", res2)
else:
    print("FAILED")
