# SPDX-FileCopyrightText: 2025 Hiroto Miura
# SPDX-License-Identifier: BSD-3-Clause

import rclpy 
from rclpy .node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

class PitchClient(Node):
    def __init__(self):
        super().__init__('pitch_client')
        
        self.declare_parameter('pitcher_name', 'Collie')
        self.pitcher_name = self.get_parameter('pitcher_name').get_parameter_value().string_value
        self.cli = self.create_client(Trigger, '/count_pitch')

        # 投手名選択用のパブリッシャ
        self.pub_select = self.create_publisher(String, '/pitch/select', 10)


        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.timer = self.create_timer(1.0, self.send_request)

    def send_selection(self):
        msg = String()
        msg.data = self.pitcher_name
        self.pub_select.publish(msg)

    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Server Response: Success={response.success}, Message="{response.message}"')
        except Exception as e:
            self.get_logger().error(f'Server call failed: {e}')


def main():
    rclpy.init()
    node = PitchClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



