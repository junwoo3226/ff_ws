import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ConveyorControlNode(Node):
    def __init__(self):
        super().__init__('conveyor_control')
        self.subscription = self.create_subscription(
            String,
            'conveyor_topic',
            self.listener_callback,
            10
        )
        self.get_logger().info("Conveyor control node started.")

    def listener_callback(self, msg):
        command = msg.data
        if command.startswith("move"):
            try:
                steps = int(command.split()[1])
                self.get_logger().info(f"Moving conveyor {steps} steps.")
                self.move_conveyor(steps)
            except (ValueError, IndexError):
                self.get_logger().error(f"Invalid command: {command}")

    def move_conveyor(self, steps):
        # 컨베이어 제어 로직 구현
        # 예: Arduino에 시리얼로 데이터 전송
        self.get_logger().info(f"Conveyor moved {steps} steps.")  # 실제 제어 코드로 대체

def main():
    rclpy.init()
    node = ConveyorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
