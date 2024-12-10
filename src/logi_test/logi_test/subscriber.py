import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_subscriber')
        self.subscription = self.create_subscription(
            String,
            'robot_topic',
            self.listener_callback,
            10  # QoS 설정
        )
        self.get_logger().info("Robot Subscriber Node has been started.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")

class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'conveyor_topic',
            self.listener_callback,
            10  # QoS 설정
        )
        self.get_logger().info("Conveyor Subscriber Node has been started.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")


def main():
    rclpy.init()

    input_num = int(input("숫자를 입력하세요. : "))


    # 노드 실행
    if input_num == 1:
        robot_subscriber = RobotNode()
        try:
            rclpy.spin(robot_subscriber)
        except KeyboardInterrupt:
            pass
    else:
        conveyor_subscriber = ConveyorNode()
        try:
            rclpy.spin(conveyor_subscriber)
        except KeyboardInterrupt:
            pass

    # 노드 종료
    robot_subscriber.destroy_node()
    conveyor_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
