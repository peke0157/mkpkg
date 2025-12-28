# SPDX-FileCopyrightText: 2025 Hiroto Miura
# SPDX-License-Identifier: BSD-3-Clause

import rclpy 
from rclpy .node import Node
from std_srvs.srv import Trigger

class PitchClient(Node):
    def __init__(self):
        super().__init__('pitch_client')
        self.cli = self.create_client(Trigger, '/count_pitch')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get.logger().info('Service not available, waiting again...')


        self.timer = self.create_timer(1.0, self.send_request)

    def send_request(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(seld.reponse_callback)

    def response_callback(self, future):
        try:
            response = future.result
            self.get_logger().info(f'Server Response: Success={response.success}, Message="{response.message}"')
        except Exception as e:
            self.get_logger().error('Server call failed: {e}')


def main(args=Node):
    rclpy.init(args=args)
    node = PitchClient
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()



