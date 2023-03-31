# Copyright (c) 2023 Aditya Kamath
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
from rclpy.node import Node
from rclpy.time import Time
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState

class MeshPublisher(object):
    def __init__(self, node, mesh_topic='mesh_array', period=0.2):
        self._node = node
        
        self._marker_pub = self._node.create_publisher(MarkerArray, mesh_topic, 8)
        self._js_init_pub = self._node.create_publisher(JointState, 'joint_states', 1)
        
        self._node.create_timer(period, self.cb_timer)
        
        self._marker_array_msg = MarkerArray()
        self._num_markers      = 8
        
        self._node.declare_parameter('frame_base',     'base_link')
        self._node.declare_parameter('frame_nav',      'nav_link')
        self._node.declare_parameter('frame_laser',    'laser_link')
        self._node.declare_parameter('frame_t265',     't265_pose_frame')
        self._node.declare_parameter('frame_wheel_lf', 'wheel_lf')
        self._node.declare_parameter('frame_wheel_lb', 'wheel_lb')
        self._node.declare_parameter('frame_wheel_rf', 'wheel_rf')
        self._node.declare_parameter('frame_wheel_rb', 'wheel_rb')
        
        self._frame_base      = self._node.get_parameter('frame_base').get_parameter_value().string_value
        self._frame_nav       = self._node.get_parameter('frame_nav').get_parameter_value().string_value
        self._frame_laser     = self._node.get_parameter('frame_laser').get_parameter_value().string_value
        self._frame_t265      = self._node.get_parameter('frame_t265').get_parameter_value().string_value
        self._frame_wheel_lf  = self._node.get_parameter('frame_wheel_lf').get_parameter_value().string_value
        self._frame_wheel_lb  = self._node.get_parameter('frame_wheel_lb').get_parameter_value().string_value
        self._frame_wheel_rf  = self._node.get_parameter('frame_wheel_rf').get_parameter_value().string_value
        self._frame_wheel_rb  = self._node.get_parameter('frame_wheel_rb').get_parameter_value().string_value
        
        self._node.declare_parameter('url_base',      "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/akros2/base_module.dae")
        self._node.declare_parameter('url_nav',      "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/akros2/navigation_module.dae")
        self._node.declare_parameter('url_laser',     "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/akros2/ld06.dae")
        self._node.declare_parameter('url_t265',      "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/akros2/t265.dae")
        self._node.declare_parameter('url_wheel_lf',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/akros2/wheel_lf.dae")
        self._node.declare_parameter('url_wheel_lb',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/akros2/wheel_lb.dae")
        self._node.declare_parameter('url_wheel_rf',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/akros2/wheel_rf.dae")
        self._node.declare_parameter('url_wheel_rb',  "https://raw.githubusercontent.com/adityakamath/akros_3d_assets/akros2/wheel_rb.dae")
        
        self._url_base      = self._node.get_parameter('url_base').get_parameter_value().string_value
        self._url_nav       = self._node.get_parameter('url_nav').get_parameter_value().string_value
        self._url_laser     = self._node.get_parameter('url_laser').get_parameter_value().string_value
        self._url_t265      = self._node.get_parameter('url_t265').get_parameter_value().string_value
        self._url_wheel_lf  = self._node.get_parameter('url_wheel_lf').get_parameter_value().string_value
        self._url_wheel_lb  = self._node.get_parameter('url_wheel_lb').get_parameter_value().string_value
        self._url_wheel_rf  = self._node.get_parameter('url_wheel_rf').get_parameter_value().string_value
        self._url_wheel_rb  = self._node.get_parameter('url_wheel_rb').get_parameter_value().string_value
        
        self._opacity = 0.95
        
        self._nearlyBlack   = ColorRGBA()
        self._nearlyBlack.r = 0.125
        self._nearlyBlack.g = 0.125
        self._nearlyBlack.b = 0.125
        self._nearlyBlack.a = 1.0

        self._darkGrey   = ColorRGBA()
        self._darkGrey.r = 0.25
        self._darkGrey.g = 0.25
        self._darkGrey.b = 0.25
        self._darkGrey.a = 1.0
        
        self._grey   = ColorRGBA()
        self._grey.r = 0.5
        self._grey.g = 0.5
        self._grey.b = 0.5
        self._grey.a = 1.0
        
        self._greyTransparent   = ColorRGBA()
        self._greyTransparent.r = 0.75
        self._greyTransparent.g = 0.75
        self._greyTransparent.b = 0.75
        self._greyTransparent.a = self._opacity
        
        for i in range(self._num_markers):
            self._marker = Marker()
            #self._marker.header.stamp = Time.now().to_msg()
            self._marker.ns = ""
            self._marker.type = 10
            self._marker.id = i
            self._marker.action = 0
            self._marker.mesh_use_embedded_materials = False
            self._marker.scale.x = 1.0
            self._marker.scale.y = 1.0
            self._marker.scale.z = 1.0
            self._marker.pose.position.x = 0.0
            self._marker.pose.position.y = 0.0
            if i==1:
                self._marker.pose.position.z = -0.099
            else:
                self._marker.pose.position.z = 0.0
            self._marker.pose.orientation.x = 0.0
            self._marker.pose.orientation.y = 0.0
            if i==2:
                self._marker.pose.orientation.z = -0.7071068
                self._marker.pose.orientation.w = 0.7071068
            else:
                self._marker.pose.orientation.z = 0.0
                self._marker.pose.orientation.w = 0.1
            self._marker.frame_locked = True
            if   i==0:
                self._marker.header.frame_id = self._frame_base
                self._marker.mesh_resource = self._url_base
                self._marker.color = self._greyTransparent
            elif i==1:
                self._marker.header.frame_id = self._frame_nav
                self._marker.mesh_resource = self._url_nav
                self._marker.color = self._darkGrey
            elif i==2:
                self._marker.header.frame_id = self._frame_laser
                self._marker.mesh_resource = self._url_laser
                self._marker.color = self._nearlyBlack
            elif i==3:
                self._marker.header.frame_id = self._frame_t265
                self._marker.mesh_resource = self._url_t265
                self._marker.color = self._grey
            elif i==4:
                self._marker.header.frame_id = self._frame_wheel_lf
                self._marker.mesh_resource = self._url_wheel_lf
                self._marker.color = self._darkGrey
            elif i==5:
                self._marker.header.frame_id = self._frame_wheel_lb
                self._marker.mesh_resource = self._url_wheel_lb
                self._marker.color = self._darkGrey
            elif i==6:
                self._marker.header.frame_id = self._frame_wheel_rf
                self._marker.mesh_resource = self._url_wheel_rf
                self._marker.color = self._darkGrey
            elif i==7:
                self._marker.header.frame_id = self._frame_wheel_rb
                self._marker.mesh_resource = self._url_wheel_rb
                self._marker.color = self._darkGrey

            self._marker_array_msg.markers.append(self._marker);
        
    def cb_timer(self):
        self._marker_pub.publish(self._marker_array_msg)
        
    def publish_joint_states(self):
        joint_state_msg = JointState()
        
        joint_state_msg.header.stamp = self._node.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = self._frame_base
        joint_state_msg.name = ['joint_lf', 'joint_rf', 'joint_lb', 'joint_rb']
        joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]
        joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]
        joint_state_msg.effort = [0.0, 0.0, 0.0, 0.0]

        self._js_init_pub.publish(joint_state_msg)

def main():
    rclpy.init()
    node = rclpy.create_node("mesh_publisher")
    mesh_publisher = MeshPublisher(node)
    mesh_publisher.publish_joint_states()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()