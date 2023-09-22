import sys

from interface.srv import FullName
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(FullName, 'summ_full_name')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FullName.Request()

    def send_request(self, a, b, c):
        self.req.first_name = a
        self.req.name = b
        self.req.last_name = c
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    client_name = MinimalClientAsync()
    response = client_name.send_request(str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]))
    client_name.get_logger().info(
        'Result of summ_full_name: for %s, %s, %s = %s' %
        (str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]), response.full_name))

    client_name.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
