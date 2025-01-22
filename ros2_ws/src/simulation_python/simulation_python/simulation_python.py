# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from aetos_msgs.msg import Joy


class SimulationPython(Node):

    def __init__(self):
        super().__init__('simulation_python')
        self.subscription = self.create_subscription(
            Joy,
            'aetos/joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.vx)


def main(args=None):
    rclpy.init(args=args)

    simulation_python = SimulationPython()

    rclpy.spin(simulation_python)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simulation_python.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    

# ros2 topic pub /aetos/joy aetos_msgs/msg/Joy "vx: 1.0 vy: 2.0 vz: 3.0"
