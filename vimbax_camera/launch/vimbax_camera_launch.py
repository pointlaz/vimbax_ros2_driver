# Copyright (c) 2024 Allied Vision Technologies GmbH. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Allied Vision Technologies GmbH nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os

import yaml

from launch import LaunchDescription
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    vimbax_camera_ld = LaunchDescription()

    share_directory_path = get_package_share_directory("vimbax_camera")
    config_path = os.path.join(
        share_directory_path,
        'config',
        'params.yaml'
    )
    camera_ids_config_path = os.path.join(
        share_directory_path,
        'config',
        'camera_ids.yaml'
    )

    with open(camera_ids_config_path, 'r') as file:
        cameras = yaml.safe_load(file)
    
    # Extract the list of cameras
    cameras_id = cameras['cameras_id']
    id_count = 0

    for camera_id in cameras_id:

        camera_node_name = f'camera_{id_count}'
        vimbax_camera_node = Node(
                package='vimbax_camera',
                namespace='',
                executable='vimbax_camera_node',
                name= camera_node_name,
                parameters=[
                config_path,
                {'camera_id' : camera_id},
                ])
        id_count += 1
        vimbax_camera_ld.add_action(vimbax_camera_node)

    return vimbax_camera_ld
