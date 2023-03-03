# Copyright (c) 2022 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3

import rclpy
import time
from visualization_msgs.msg import Marker, MarkerArray
from akros2_msgs.msg import Velocities
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class MeshPublisher(object):
    def __init__(self, node, vels_topic='velocities', mesh_topic='mesh_array', joint_state_topic='joint_states', period=0.2):
        self._node = node
       
        self._node.create_subscription(Velocities, vels_topic, self.cb_vels, 1)
        self._marker_pub = self._node.create_publisher(MarkerArray, mesh_topic, 8)
        self._state_pub  = self._node.create_publisher(JointState, joint_state_topic, 8)
        self._node.create_timer(period, self.cb_timer)
        
        self._raw_vels        = Velocities()
        self._joint_state_msg = JointState()
        self._velocities      = [0.0, 0.0, 0.0, 0.0] # lf, lb, rf, rb
        self._positions       = [0.0, 0.0, 0.0, 0.0] # lf, lb, rf, rb
        self._vel_lf          = 0.0
        self._vel_lb          = 0.0
        self._vel_rf          = 0.0
        self._vel_rb          = 0.0
        self._pos_lf          = 0.0
        self._pos_lb          = 0.0
        self._pos_rf          = 0.0
        self._pos_rb          = 0.0
        
        self._marker_array_msg = MarkerArray()
        self._num_markers      = 8
        
        self._frame_base      = rospy.get_param('frame_base',      'base_link')
        self._frame_laser     = rospy.get_param('frame_laser',     'laser_link')
        self._frame_t265      = rospy.get_param('frame_t265',      't265_pose_frame')
        self._frame_footprint = rospy.get_param('frame_footprint', 'base_footprint')
        self._frame_wheel_lf  = rospy.get_param('frame_wheel_lf',  'wheel_lf')
        self._frame_wheel_lb  = rospy.get_param('frame_wheel_lb',  'wheel_lb')
        self._frame_wheel_rf  = rospy.get_param('frame_wheel_rf',  'wheel_rf')
        self._frame_wheel_rb  = rospy.get_param('frame_wheel_rb',  'wheel_rb')
        
        self._url_base      = rospy.get_param('url_base',      "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/navigation_module_centered.stl")
        self._url_laser     = rospy.get_param('url_laser',     "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/ld06.stl")
        self._url_t265      = rospy.get_param('url_t265',      "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/t265.stl")
        self._url_footprint = rospy.get_param('url_footprint', "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/base_module_centered.stl")
        self._url_wheel_lf  = rospy.get_param('url_wheel_lf',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/wheel_left_front.stl")
        self._url_wheel_lb  = rospy.get_param('url_wheel_lb',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/wheel_left_back.stl")
        self._url_wheel_rf  = rospy.get_param('url_wheel_rf',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/wheel_right_front.stl")
        self._url_wheel_rb  = rospy.get_param('url_wheel_rb',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/main/wheel_right_back.stl")
        
        for i in range(self._num_markers):
            self._marker = Marker()
            self._marker.header.stamp = rospy.Time.now()
            self._marker.ns = ""
            self._marker.type = 10
            self._marker.id = i
            self._marker.action = 0
            self._marker.mesh_use_embedded_materials = False
            self._marker.scale.x = 1
            self._marker.scale.y = 1
            self._marker.scale.z = 1
            self._marker.pose.position.x = 0
            self._marker.pose.position.y = 0
            self._marker.pose.position.z = 0
            self._marker.pose.orientation.x = 0.0
            self._marker.pose.orientation.y = 0.0
            self._marker.pose.orientation.z = 0.0
            self._marker.pose.orientation.w = 1.0
            self._marker.frame_locked = True
            if i==0 or i==3:
                self._marker.color.r = 1.0
                self._marker.color.g = 1.0
                self._marker.color.b = 1.0
                self._marker.color.a = 0.75
            else:
                self._marker.color.r = 0.2078
                self._marker.color.g = 0.2549
                self._marker.color.b = 0.2784
                self._marker.color.a = 0.95
            if i==0:
                self._marker.header.frame_id = self._frame_base
                self._marker.mesh_resource = self._url_base
            elif i==1:
                self._marker.header.frame_id = self._frame_laser
                self._marker.mesh_resource = self._url_laser
            elif i==2:
                self._marker.header.frame_id = self._frame_t265
                self._marker.mesh_resource = self._url_t265
            elif i==3:
                self._marker.header.frame_id = self._frame_footprint
                self._marker.mesh_resource = self._url_footprint
            elif i==4:
                self._marker.header.frame_id = self._frame_wheel_lf
                self._marker.mesh_resource = self._url_wheel_lf
            elif i==5:
                self._marker.header.frame_id = self._frame_wheel_lb
                self._marker.mesh_resource = self._url_wheel_lb
            elif i==6:
                self._marker.header.frame_id = self._frame_wheel_rf
                self._marker.mesh_resource = self._url_wheel_rf
            elif i==7:
                self._marker.header.frame_id = self._frame_wheel_rb
                self._marker.mesh_resource = self._url_wheel_rb

            self._marker_array_msg.markers.append(self._marker);
        
    def cb_timer(self):
        self._velocities[0] = self._raw_vels.motor1 #lf
        self._velocities[1] = self._raw_vels.motor2 #lb
        self._velocities[3] = self._raw_vels.motor3 #rb
        self._velocities[2] = self._raw_vels.motor4 #rf
        
        for j in range(4):
                self._positions[j] += self._velocities[j]/5 #1/5 second frequency
                #limit position between -pi and pi
                if self._positions[j] > math.pi: self._positions[j] = -1*math.pi
                if self._positions[j] < -1*math.pi: self._positions[j] = math.pi
                
        self._joint_state_msg.header = Header()
        self._joint_state_msg.header.frame_id = self._frame_footprint
        self._joint_state_msg.header.stamp = rospy.Time.now()
        self._joint_state_msg.name = ['joint_lf', 'joint_lb', 'joint_rf', 'joint_rb']
        self._joint_state_msg.velocity = self._velocities
        self._joint_state_msg.position = self._positions
        self._joint_state_msg.effort   = []
        
        self._state_pub.publish(self._joint_state_msg)
        self._marker_pub.publish(self._marker_array.msg)

    def cb_vels(self, msg):
        """
        :type msg: Velocities
        """
        self._raw_vels = msg

def main():
    rclpy.init()
    node = rclpy.create_node("mesh_publisher")
    MeshPublisher(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()