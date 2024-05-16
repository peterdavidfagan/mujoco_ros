# mujoco_ros
MuJoCo simulation instances that are compatible with the ROS 2 ecosystem.

## Sample

```python 
import os
import time

import mujoco
import mujoco.viewer

import mujoco_ros
from mujoco_ros.franka_env import FrankaEnv

model_filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models', 'franka_table.mjb')

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
```

[Screencast from 05-16-2024 01_56_12 PM.webm](https://github.com/peterdavidfagan/mujoco_ros/assets/42982057/f19f863e-5251-403a-8234-d94907f19fd4)


## Motion Planning to Random Configuration with Moveit
[Screencast from 02-08-2024 06:27:55 PM.webm](https://github.com/peterdavidfagan/mujoco_ros_sim/assets/42982057/eb122924-b62e-4980-9003-01e72316f4af)

Tutorial code to be release soon under [ros2_robotics_research_toolkit](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit).
