import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

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
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        super().__init__('conveyor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'conveyor_topic',
            self.listener_callback,
            10  # QoS 설정
        )
        self.serial_port = serial.Serial(port, baudrate)  # 시리얼 초기화
        self.get_logger().info("Conveyor Subscriber Node has been started.")

    def listener_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        if command.startswith("move"):
            try:
                steps = int(command.split()[1])
                self.get_logger().info(f"Moving conveyor {steps} steps.")
                self.move_conveyor(steps)
            except (ValueError, IndexError):
                self.get_logger().error(f"Invalid command: {command}")

    def move_conveyor(self, steps):
        # 실제 하드웨어 명령 예시 (시리얼 통신 등)
        try:
            # 시리얼로 명령 전송
            command = f"move {steps}\n"
            self.serial_port.write(command.encode())
            self.get_logger().info(f"Sent to Arduino: {command}")
        except Exception as e:
            self.get_logger().error(f"Error moving conveyor: {str(e)}")


def main():
    rclpy.init()

    input_num = int(input("숫자를 입력하세요 (1: Robot Node, 2: Conveyor Node): "))

    if input_num == 1:
        # RobotNode 실행
        robot_subscriber = RobotNode()
        try:
            rclpy.spin(robot_subscriber)
        except KeyboardInterrupt:
            pass
        finally:
            robot_subscriber.destroy_node()
    elif input_num == 2:
        # ConveyorNode 실행
        conveyor_subscriber = ConveyorNode()
        try:
            rclpy.spin(conveyor_subscriber)
        except KeyboardInterrupt:
            pass
        finally:
            conveyor_subscriber.destroy_node()
    else:
        print("잘못된 입력입니다. 1 또는 2를 입력해주세요.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
