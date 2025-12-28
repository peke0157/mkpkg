# SPDX-FileCopyrightText: 2025 Hiroto Miura
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class PitchServer(Node):
    def __init__(self):
        super().__init__('pitch_server')
        self.declare_parameter('start_pitcher', 'Collie')
        
        self.declare_parameter('pitch_limit', 100)      # 投球制限100球
        # 投手管理リスト
        self.current_count = {}
        self.current_pitcher = self.get_parameter('start_pitcher').get_parameter_value().string_value       # 現在選択されている選手

        # 外部入力でparameter_callbackを実行させる
        self.add_on_set_parameters_callback(self.parameter_callback)


        self.current_count[self.current_pitcher] = 0

        # --- パブリッシャーの作成 ---
        self.pub = self.create_publisher(String, '/pitch/warning', 10)

        # --- サブスクライバの作成 ---
        self.create_subscription(String, '/pitch/select', self.select_picher_callback, 10)
        
        # --- サービス(カウント用) ---
        self.srv = self.create_service(Trigger, '/count_pitch', self.check_pitch_callback)
        
        self.srv = self.create_service(Trigger, '/reset_pitch', self.reset_callback)

        self.get_logger().info('Pitch Server Ready')
        self.get_logger().info(f"Start Pitcher is set to: {self.current_pitcher}")


    def check_pitch_callback(self, request, response):
        # 球数を増やす
        self.current_count[self.current_pitcher] += 1

        # 球数を取得
        limit = self.get_parameter('pitch_limit').get_parameter_value().integer_value

        self.get_logger().info(f'Pitch_Count: {self.current_count[self.current_pitcher]}/{limit}')

        if self.current_count[self.current_pitcher] > limit:
            msg = String()
            msg.data = f"Warning: pitch_count_limit exceed! ({self.current_count[self.current_pitcher]}/{limit})"
            self.pub.publish(msg)

            response.success = False
            response.message = f"Limited Exceed! count = ({self.current_count[self.current_pitcher]})"

        else:
            response.success = True
            response.message = f"OK"

        return response        

    
    def select_picher_callback(self, msg):
        name = msg.data
        self.current_picher = name
        if name not in self.current_count:
            self.current_count[name] = 0
        
        limit = self.get_parameter('pitch_limit').get_parameter_value().integer_value

        self.get_logger().info(f'Selected Pitcher: {name} (Current Count: {self.current_count[self.current_pitcher]}/{limit})')

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'start_pitcher':
                new_name = param.get_parameter_value().string_value
                self.current_pitcher = new_name

                self.get_logger().info(f'Pitcher changed to: {self.current_pitcher}')
                
                self.current_count[self.current_pitcher] = 0
        
        from rcl_interfaces.msg import SetParametersResult
        return SetParametersResult(successful=True)


    def reset_callback(self, request, response):
        self.current_count[self.current_pitcher] = 0
        self.get_logger().info('Pitch_Count_Reset')
        response.success = True
        response.message = "Counter has been reset."
        return response


def main():
    rclpy.init()
    node = PitchServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()

