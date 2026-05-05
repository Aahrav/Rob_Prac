import numpy as np
import logging
from backend.kinematics import KinematicChain, DHJoint

logging.basicConfig(level=logging.INFO)

# Hypothesis: J1 has a=0, d=0.26. J2 has a=0.26, d=0. J3 has a=0.18, d=0. J4 is prismatic.
joints = [
    DHJoint('revolute', theta=0.0, d=0.26, a=0.0, alpha=0.0, name="Base Yaw", q_min=-180.0, q_max=180.0),
    DHJoint('revolute', theta=0.0, d=0.0,  a=0.26, alpha=0.0, name="Elbow Yaw", q_min=-180.0, q_max=180.0),
    DHJoint('revolute', theta=0.0, d=0.0,  a=0.18, alpha=0.0, name="Wrist Yaw", q_min=-180.0, q_max=180.0),
    DHJoint('prismatic', theta=0.0, d=0.0, a=0.0,  alpha=0.0, name="Vertical (Z)", q_min=0.0, q_max=0.18),
]
chain = KinematicChain(joints, base_height=0.0)

target = np.array([-0.2580, -0.5660, 0.2050])
sol = chain.inverse_kinematics(target, max_iter=200)
print("Solution:", sol)
