import numpy as np
from backend.robot_presets import preset_stanford_arm

chain = preset_stanford_arm()
targets = [
    np.array([0.1050, 0.0000, 0.2050]),
    np.array([-0.2050,-0.3320,0.1500]),
    np.array([0.1000,0.1000,0.1500]),
    np.array([0.8830,-0.4030,0.2050])
]

# extract ik logic to patch it locally
def run_custom_ik(chain, target_position, tol=0.005, max_iter=200):
    var_indices = [i for i, j in enumerate(chain.joints) if j.type in ('revolute', 'prismatic')]
    angles = [0.0]*len(chain.joints)
    best_error = float('inf')
    best_angles = list(angles)
    
    damping = 0.01
    max_step_deg = 15.0
    max_step_m = 0.05
    
    for it in range(max_iter):
        transforms = chain.forward_kinematics(angles)
        ee_pos = transforms[-1][:3, 3]
        error = target_position - ee_pos
        err_norm = np.linalg.norm(error)
        
        if err_norm < best_error:
            best_error = err_norm
            best_angles = list(angles)
            
        if err_norm < tol:
            return best_angles, err_norm
            
        J = np.zeros((3, len(var_indices)))
        for col, idx in enumerate(var_indices):
            perturbed = list(angles)
            if chain.joints[idx].type == 'revolute':
                step_val = 1.0 # 1 deg
                perturbed[idx] += step_val
                pos_plus = chain.forward_kinematics(perturbed)[-1][:3, 3]
                J[:, col] = (pos_plus - ee_pos) / np.radians(step_val)
            else:
                step_val = 0.01 # 1 cm
                perturbed[idx] += step_val
                pos_plus = chain.forward_kinematics(perturbed)[-1][:3, 3]
                J[:, col] = (pos_plus - ee_pos) / step_val
                
        JJT = J @ J.T
        try:
            delta_vars = J.T @ np.linalg.solve(JJT + damping**2 * np.eye(3), error)
        except:
            break
            
        for col, idx in enumerate(var_indices):
            step = delta_vars[col]
            if chain.joints[idx].type == 'revolute':
                step_deg = np.degrees(step)
                step_deg = np.clip(step_deg, -max_step_deg, max_step_deg)
                angles[idx] += step_deg
            else:
                step = np.clip(step, -max_step_m, max_step_m)
                angles[idx] += step
                
            j = chain.joints[idx]
            if j.q_min is not None: angles[idx] = max(angles[idx], j.q_min)
            if j.q_max is not None: angles[idx] = min(angles[idx], j.q_max)
            
    return best_angles, best_error

for t in targets:
    res, err = run_custom_ik(chain, t)
    print(f"Target {t} -> err {err}")
