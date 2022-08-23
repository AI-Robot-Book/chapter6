import rclpy
from rclpy.node import Node
from airobot_interfaces.srv import StringCommand


class TestClient(Node):

    def __init__(self):
        super().__init__('test_client')
        self.client = self.create_client(StringCommand, 'manipulation/command')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('サービス無効，待機中...')

    def send_request(self, command):
        request = StringCommand.Request()
        request.command = command
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().answer


def main():
    rclpy.init()

    node = TestClient()

    try:
        while True:
            command = input('command: ')
            if command == '':
                break
            answer = node.send_request(command)
            print(f'answer: {answer}')
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
