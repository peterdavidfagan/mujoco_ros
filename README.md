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

## ROS 2 Control Joint Trajectory Controller + MoveIt 2
Please note that the controller isn't perfectly tuned for the simulation but this is provided as an example of the interactivity you can accomplish with this package.

[Screencast from 05-30-2024 11:17:53 AM.webm](https://github.com/peterdavidfagan/mujoco_ros/assets/42982057/df2b1548-aab1-4a06-b65c-d4aa70812be2)
