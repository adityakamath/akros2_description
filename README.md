# akros2_description
Package with AKROS2 description files for different configurations - Mecanum Drive (4 wheeled), Omni-Wheel Drive (3 wheeled)

* Publishes the robot description using [robot_state_publisher](https://github.com/ros/robot_state_publisher) and publishes the corresponding joint states using [joint_state_publisher](https://github.com/ros/joint_state_publisher). 
* The ```joint_state_publisher``` subscribes to [JointState messages](https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html) for each defined joint in the launch file. It then publishes a combined JointState message, which the ```robot_state_publisher``` subscribes to and updates the transforms.
* For visualization using some apps, the ```*.urdf.xacro``` should be converted to a ```*.urdf``` file using the ```xacro``` command from the root of this directory. For example:
  
  ```
  xacro urdf/akros2_<config>/robot.urdf.xacro nopath:=False > urdf/akros2_<config>/robot.urdf
  ```
  This argument fixes the launch path in the generated .urdf file so that it can be used with apps like Unity or Foxglove. The generated URDF should be copied into the [akros_3d_assets](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf) repo, in the corresponding mecanum/omni directory (replacing the existing .urdf file if there is one already). This directory must be copied into the ```Assets``` directory in the Unity project. Then, from Unity, the URDF can be imported into the project as a GameObject using the [Unity-ROS](https://github.com/Unity-Technologies/Unity-Robotics-Hub) [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer#integrate-urdf-importer-into-unity-project).
* For visualization on other tools/apps (other than Unity), meshes can also be referenced from the corresponding directory of a remote repository [akros_3d_assets (```akros2_urdf``` branch)](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf) by updating the ```mesh_path``` property in ```akros2_<config>.urdf.xacro```. While this works on RViz and Foxglove Studio, some apps might not be able to parse meshes from remote URLs, so the local meshes are currently used as default.
* The [akros_3d_assets repo (```akros2_urdf``` branch)](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf) also contains ```akros2_<config>.urdf``` in the config-corresponding directory, which can also be used directly in apps like Foxglove Studio.

## description_launch.py
This is the main launch file and is configured using the following launch arguments:

* ```config```: Configures the robot description based on the platform: ```mecanum``` (4-wheeled) or ```omni``` (3-wheeled) (Default: ```mecanum```)
* ```js_ext```: If True, enables joint states from external nodes (like a [micro-ROS node](https://github.com/adityakamath/akros2_firmware/tree/akros2_humble)). If False, enables joint states from the joint_state_publisher (Default: ```True```)
* ```js_topic```: Choose whether to report measured joint states ```joint_states```, or required joint states ```req_states``` (Default: ```joint_states```)

