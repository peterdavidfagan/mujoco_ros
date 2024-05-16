import os
import time

import mujoco
import mujoco.viewer

import mujoco_ros
from mujoco_ros.franka_env import FrankaEnv

model_filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models', 'franka_table.mjb')
print(model_filepath)

if __name__=="__main__":
    m = mujoco.MjModel.from_binary_path(model_filepath)
    d = mujoco.MjData(m)
    with mujoco.viewer.launch_passive(
        model=m, 
        data=d,
        show_left_ui=False,
        show_right_ui=False,
        ) as viewer:
        # instaniate mujoco_ros environment
        env = FrankaEnv(m, d)

        # run interactive viewer application
        while viewer.is_running():
            time.sleep(0.01)

            # for now we need to let env know that we are syncing
            # TODO: write basic wrapper for viewer that handles this
            env.is_syncing = True
            viewer.sync()
            env.is_syncing = False
