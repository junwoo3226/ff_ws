import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smtplib
from email.mime.text import MIMEText
import serial
import threading
import time
from datetime import datetime

import subprocess

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



def set_permissions():
    try:
        result = subprocess.run(
            ["sudo", "chmod", "666", "/dev/ttyACM0"],
            check=True,
            text=True,
            capture_output=True
        )
        print("권한 변경 성공:", result.stdout)
    except subprocess.CalledProcessError as e:
        print("권한 변경 실패:", e.stderr)
        
class ConveyorNode(Node):
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        super().__init__('conveyor_subscriber')
        self.subscription = self.create_subscription(
            String,
            'conveyor_topic',
            self.listener_callback,
            10  # QoS 설정
        )

        # 시리얼 포트 초기화 (write_timeout 설정 추가)
        self.serial_port = serial.Serial(
            port,
            baudrate,
            timeout=1,         # 읽기 타임아웃
            write_timeout=2,   # 쓰기 타임아웃
            dsrdtr=False,      # DSR/DTR 비활성화
            rtscts=False       # RTS/CTS 비활성화
        )
        self.get_logger().info("Conveyor Subscriber Node has been started.")

    def listener_callback(self, msg):
        set_permissions()
        command = msg.data
        self.get_logger().info(f"Received command: {command}")
        if command.startswith("move"):
            
            try:
                steps = int(command.split()[1])
                self.get_logger().info(f"Moving conveyor {steps} steps.")
                self.move_conveyor(steps)
            except (ValueError, IndexError):
                error_message = f"Invalid command received: {command}"
                self.get_logger().error(error_message)
                self.send_email(error_message)

    def move_conveyor(self, steps):
        
        set_permissions()
        try:
            current_step = 0
            while current_step < steps:
                # 시리얼로 명령 전송
                command = f"move {current_step}\n"
                self.serial_port.write(command.encode())
                self.get_logger().info(f"Sent to Arduino: {command}")

                # steps를 2씩 증가
                current_step += 2

                # USB 끊김이나 다른 문제 발생 시 처리
                time.sleep(0.1)  # 약간의 딜레이 추가
        except (serial.SerialTimeoutException, serial.SerialException, OSError) as e:
            error_message = f"USB 연결이 끊겼습니다: {str(e)}"
            self.get_logger().error(error_message)
            self.reconnect_serial()
                    # 이메일 전송
            email_thread = threading.Thread(target=self.send_email, args=(error_message,))
            email_thread.start()
            
        except Exception as e:
            error_message = f"Error moving conveyor: {str(e)}"
            self.get_logger().error(error_message)
                    # 이메일 전송
            email_thread = threading.Thread(target=self.send_email, args=(error_message,))
            email_thread.start()




    def reconnect_serial(self):
        """USB 연결 재시도"""
        self.get_logger().info("USB 연결 재시도 중...")
        retry_count = 0
        max_retries = 5
        set_permissions()

        while retry_count < max_retries:
            
            set_permissions()
            try:
                self.serial_port.close()  # 기존 포트 닫기
                self.serial_port.open()  # 다시 열기
                self.get_logger().info("USB 연결 성공")
                return
            except (serial.SerialException, OSError) as e:
                retry_count += 1
                self.get_logger().warning(f"USB 연결 재시도 {retry_count}/{max_retries}: {str(e)}")
                time.sleep(2)  # 재시도 전 대기
        error_message = "USB 연결 재시도 실패. 수동 확인 필요."
        self.get_logger().error(error_message)

        # 이메일 전송
        email_thread = threading.Thread(target=self.send_email, args=(error_message,))
        email_thread.start()

    def send_email(self, error_message):
        """오류 메시지를 이메일로 전송"""
        smtp_server = "smtp.gmail.com"
        smtp_port = 587
        sender_email = "a12012032@gmail.com"
        sender_password = "xyoz mbkb fcdr lnrm"  # 보안을 위해 환경 변수로 관리 권장
        recipient_email = "a12012032@gmail.com"  # 받는 사람 이메일

        subject = "[컨베이어 오류] 알림"
        body = (
            f"컨베이어에서 오류가 발생했습니다.\\n\\n"
            f"오류 메시지: {error_message}\\n"
            f"발생 시간: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
        )

        try:
            msg = MIMEText(body, _charset="utf-8")
            msg["Subject"] = subject
            msg["From"] = sender_email
            msg["To"] = recipient_email
            self.get_logger().info("SMTP 서버에 연결 중...")
            server = smtplib.SMTP(smtp_server, smtp_port)
            server.starttls()
            server.login(sender_email, sender_password)
            self.get_logger().info("SMTP 로그인 성공")
            server.sendmail(sender_email, recipient_email, msg.as_string())
            server.quit()
            self.get_logger().info("오류 이메일 전송 완료")
        except Exception as e:
            self.get_logger().error(f"이메일 전송 실패: {str(e)}")


def main():
    rclpy.init()
    set_permissions()
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

