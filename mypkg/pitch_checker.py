# SPDX-FileCopyrightText: 2025 Hiroto Miura
# SPDX-License-Identifier: BSD-3-Clause

import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class PitchChecker(Node):
    def __init__(self):
        super().__init__('pitch_checker')
        self.declare_parameter('pitch_limit', 100)          # 100球投げたら警告を出す
        self.current_count = 0

        # --- パブリッシャーの作成 ---
        self.pub = self.create_publisher(String, '/pitch/warning', 10)

        self.srv = self.create_service(Trigger, '/count_pitch', self.check_pitch_callback)

    def check_pitch_callback(self, request, response):
        # 球数を増やす
        self.current_count += 1

        # 球数を取得
        limit = self.get_parameter('pitch_limit').get_parameter_value().integer_value

        self.get_logger().info(f'Pitch_Count: {self.current_count}/{limit}')

        if self.current_count > limit:
            msg = String
            msg.data = f"Warning: pitch_count_limit exceed! ({self.current_count}/{limit})"
            self.pub.publish(msg)

            response.success = False
            response.message = f"Limited Exceed! count = ({self.current_count})"

        else:
            response.success = True
            response.message = f"OK"

        return response

rclpy.init()
pitch_checker = PitchChecker()
rclpy.spin(pitch_checker)

