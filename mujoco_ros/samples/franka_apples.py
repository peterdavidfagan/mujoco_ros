import os
import time

import mujoco
import mujoco.viewer

import mujoco_ros
from mujoco_ros.franka_env import FrankaApples

model_filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models', 'apples.mjb')

if __name__=="__main__":
    m = mujoco.MjModel.from_binary_path(model_filepath)
    d = mujoco.MjData(m)

    env = FrankaApples(
            m, 
            d, 
            command_interface="effort",
            control_steps=5, 
            control_timer_freq=1e-2,
            )
    env.reset()

    with mujoco.viewer.launch_passive(
        model=m, 
        data=d,
        show_left_ui=False,
        show_right_ui=False,
        ) as viewer:
        
        # run interactive viewer application
        while viewer.is_running():
            time.sleep(0.05)
            env.is_syncing = True
            viewer.sync()
            env.is_syncing = False 
