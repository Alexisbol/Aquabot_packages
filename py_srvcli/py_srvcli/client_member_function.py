import sys

from tutorial_interfaces.srv import Pathfind
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(Pathfind, 'FindPath')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Pathfind.Request()

    def send_request(self, start, goal):
        self.req.start.x = start[0]
        self.req.start.y = start[1]
        self.req.goal.x = goal[0]
        self.req.goal.y = goal[1]
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request((float(sys.argv[1]), float(sys.argv[2])),(float(sys.argv[3]),float(sys.argv[4])))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    result_str = 'Result of pathfind: \n'
    for point in response.path.points:
        result_str += 'Point: (%.2f, %.2f)\n' % (point.x, point.y)

    minimal_client.get_logger().info(result_str)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()