# akros2_description
Package with AKROS2 description files

* Publishes the AKROS2 description using [robot_state_publisher](https://github.com/ros/robot_state_publisher) and publishes the joint states using [joint_state_publisher](https://github.com/ros/joint_state_publisher). 
* The ```joint_state_publisher``` subscribes to [JointState messages](https://docs.ros2.org/foxy/api/sensor_msgs/msg/JointState.html) for each defined joint in the launch file. It then publishes a combined JointState message, which the ```robot_state_publisher``` subscribes to and updates the transforms.
* The launch file also includes an argument ```micro_ros``` which when true will disable the ```joint_state_publisher``` node. Instead, the combined JointState message is published by the micro-ROS node using either the measured or required velocities (based on another launch argument ```joint_state_topic```) of each joint (motor).
* Uses the ```akros2.urdf.xacro``` file for visualization in RViz (default configuration provided in the rviz directory). TODO: Update the xacro files for Gazebo and ros2_control.
* For visualization on Unity, the ```akros2.urdf.xacro``` file should be converted to ```akros2.urdf``` using the ```xacro``` command from the akros2_description root:
  
  ```
  xacro urdf/akros2.urdf.xacro unity:=True > urdf/akros2.urdf
  ```
  This argument fixes the launch path in the generated .urdf file so that it can be used with Unity. The generated URDF should be copied into the [akros_3d_assets](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf) directory (replacing the existing .urdf file if there is one already). This directory must be copied into the ```Assets``` directory in the Unity project. Then, from Unity, the URDF can be imported into the project as a GameObject using the [Unity-ROS](https://github.com/Unity-Technologies/Unity-Robotics-Hub) [URDF Importer](https://github.com/Unity-Technologies/URDF-Importer#integrate-urdf-importer-into-unity-project).
* For visualization on other tools/apps (other than Unity), meshes can also be referenced from the remote repository [akros_3d_assets](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf) by updating the ```mesh_path``` property in ```akros2.urdf.xacro``` While it works on RViz, Foxglove Studio, some apps might not be able to parse meshes from remote URLs, so the local meshes are currently used as default.
* The [akros_3d_assets](https://github.com/adityakamath/akros_3d_assets/tree/akros2_urdf) also contains ```akros2.urdf```, which can also be used directly in apps like Foxglove Studio.
