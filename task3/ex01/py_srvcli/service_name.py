from interface.srv import FullName

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(FullName, 'summ_full_name', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.full_name = request.first_name + request.name + request.last_name
        self.get_logger().info('Incoming request\nfirst_name: %s name: %s last_name: %s' % (request.first_name, request.name, request.last_name))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
