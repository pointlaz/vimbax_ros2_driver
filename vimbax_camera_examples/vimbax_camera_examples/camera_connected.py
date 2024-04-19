# Copyright 2024 Allied Vision Technologies GmbH. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import vimbax_camera_msgs.srv
import argparse
from .helper import single_service_call, build_topic_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("node_namespace")

    (args, rosargs) = parser.parse_known_args()

    rclpy.init(args=rosargs)

    node = Node("_connected_get")

    # Build topic path from namespace and topic name
    topic: str = build_topic_path(args.node_namespace, '/connected')

    service_type = vimbax_camera_msgs.srv.ConnectionStatus

    request = service_type.Request()
    response = single_service_call(node, service_type, topic, request)

    if response.connected:
        print("Camera is connected!")
    else:
        print("Camera is disconnected!")