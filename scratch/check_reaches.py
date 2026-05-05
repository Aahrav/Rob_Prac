
from backend.robot_presets import PRESETS
for name, data in PRESETS.items():
    chain = data['factory']()
    print(f"{name}: {chain.get_max_reach():.4f}")
