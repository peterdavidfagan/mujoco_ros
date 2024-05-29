# mujoco_ros
MuJoCo simulation instances that are compatible with the ROS 2 ecosystem. For environment generation see [mujoco_robot_environments](https://github.com/peterdavidfagan/mujoco_robot_environments); for example use cases of this package see [ros2_robotics_research_toolkit](https://github.com/peterdavidfagan/ros2_robotics_research_toolkit).

## Example Usage

```python 
import os
import time

import mujoco
import mujoco.viewer

import mujoco_ros
from mujoco_ros.franka_env import FrankaBase

model_filepath = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'models', 'base.mjb')

if __name__=="__main__":
    m = mujoco.MjModel.from_binary_path(model_filepath)
    d = mujoco.MjData(m)

    env = FrankaBase(
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
```

## Environment Examples [MoveIt + ROS2 Control Examples to be Published Shortly]
[Screencast from 05-16-2024 01_56_12 PM.webm](https://github.com/peterdavidfagan/mujoco_ros/assets/42982057/f19f863e-5251-403a-8234-d94907f19fd4)
