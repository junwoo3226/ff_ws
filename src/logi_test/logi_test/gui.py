import sys
import cv2
import os
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QDialog, QMessageBox, QLineEdit, QFormLayout, QPushButton
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smtplib
from email.mime.text import MIMEText
from datetime import datetime


# 카메라 인덱스 확인

current_dir = os.path.dirname(os.path.abspath(__file__))
ui_path = os.path.join(current_dir, "detection.ui")
cam_idx = 0
current_time = datetime.now().strftime("[%H:%M:%S] - ")

##### ROS 2 Publisher Node #####
class ROSPublisherNode(Node):
    def __init__(self):
        super().__init__('job_publisher')
        self.robot_publisher = self.create_publisher(String, 'robot_topic', 10)
        self.conveyor_publisher = self.create_publisher(String, 'conveyor_topic', 10)

    def robot_publish_message(self, message):
        msg = String()
        msg.data = message
        self.robot_publisher.publish(msg)
        self.get_logger().info(f"Published: {message}")

    def conveyor_publish_message(self, message):
        msg = String()
        msg.data = message
        self.robot_publisher.publish(msg)
        self.get_logger().info(f"Published: {message}")


##### 로그인창 #####
class LoginDialog(QDialog):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Login")
        self.setGeometry(400, 300, 300, 150)

        # ID, Password 입력 폼
        self.id_input = QLineEdit(self)
        self.id_input.setPlaceholderText("ID")
        self.password_input = QLineEdit(self)
        self.password_input.setEchoMode(QLineEdit.Password)
        self.password_input.setPlaceholderText("Password")

        # 로그인 버튼
        self.login_button = QPushButton("Login", self)
        self.login_button.clicked.connect(self.check_credentials)

        # 레이아웃 설정
        layout = QFormLayout(self)
        layout.addRow("ID:", self.id_input)
        layout.addRow("Password:", self.password_input)
        layout.addWidget(self.login_button)

    def check_credentials(self):
        # ID와 비밀번호 확인
        user_id = self.id_input.text()
        password = self.password_input.text()

        #if user_id == "admin" and password == "123456":
        if user_id == "" and password == "":
            self.accept()  # 로그인 성공
        else:
            QMessageBox.warning(self, "Login Failed", "Invalid ID or Password")


##### 메인 애플리케이션 #####
class SimpleApp(QtWidgets.QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.elapsed_time = 0  # 오류 발생 시간

        # UI 파일 불러오기
        uic.loadUi(ui_path, self)

        # UI에서 slider, log, 카메라 QLabel, 버튼, 이메일 입력 필드 찾기
        self.conveyor_slider = self.findChild(QtWidgets.QSlider, 'conveyor_slider')  # QSlider 객체
        self.log_display = self.findChild(QtWidgets.QTextEdit, 'log_display')  # QTextBrowser 객체
        self.usbcam = self.findChild(QtWidgets.QLabel, 'usbcam')  # QLabel 객체
        self.job1_button = self.findChild(QtWidgets.QPushButton, 'job1_button')
        self.job2_button = self.findChild(QtWidgets.QPushButton, 'job2_button')
        self.job3_button = self.findChild(QtWidgets.QPushButton, 'job3_button')
        self.recipient = self.findChild(QtWidgets.QLineEdit, 'recipient')  # 이메일 입력 QLineEdit
        self.send_email_button = self.findChild(QtWidgets.QPushButton, 'send_email_button')

        self.log_display.setReadOnly(True)

        # UI 로드 확인
        if not all([self.conveyor_slider, self.log_display, self.usbcam, self.job1_button, self.job2_button,
                    self.job3_button, self.recipient, self.send_email_button]):
            print("Error: Required widgets not found in the UI file.")
            sys.exit()

        # 슬라이더 상태
        self.slider_state = None

        # 카메라 초기화
        self.cap = cv2.VideoCapture(cam_idx)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not self.cap.isOpened():
            QMessageBox.critical(self, "Error", "Failed to open camera")
            sys.exit()

        # 타이머를 사용하여 실시간 카메라 화면 업데이트
        self.camera_timer = QtCore.QTimer()
        self.camera_timer.timeout.connect(self.update_camera_frame)
        self.camera_timer.start(30)  # 30ms마다 업데이트

        # 슬라이더 이벤트 연결
        self.conveyor_slider.valueChanged.connect(self.handle_slider_change)

        # 버튼 클릭 이벤트 연결
        self.job1_button.clicked.connect(self.handle_job1)
        self.job2_button.clicked.connect(self.handle_job2)
        self.job3_button.clicked.connect(self.handle_job3)

        self.capture_button.clicked.connect(self.handle_capture)

        self.start_button.clicked.connect(self.handle_start)
        self.stop_button.clicked.connect(self.handle_stop)
        self.resume_button.clicked.connect(self.handle_resume)
        self.pause_button.clicked.connect(self.handle_pause)
        self.reset_button.clicked.connect(self.handle_reset)
        self.train_button.clicked.connect(self.handle_train)



        # 이메일 발송 버튼 이벤트 연결
        self.send_email_button.clicked.connect(self.send_email)

    ##### 슬라이더 이벤트 처리 #####
    def handle_slider_change(self):
        value = self.conveyor_slider.value()
        if value < 40:
            if self.slider_state != 'backward':
                self.log_display.append("컨베이어가 뒤로 움직입니다.")
                self.slider_state = 'backward'
                self.ros_node.conveyor_publish_message(self.slider_state)
        elif 40 <= value <= 60:
            if self.slider_state != 'stop':
                self.log_display.append("컨베이어가 정지합니다.")
                self.slider_state = 'stop'
                self.ros_node.conveyor_publish_message(self.slider_state)
        else:
            if self.slider_state != 'forward':
                self.log_display.append("컨베이어가 앞으로 움직입니다.")
                self.slider_state = 'forward'
                self.ros_node.conveyor_publish_message(self.slider_state)

    ##### 카메라 프레임 업데이트 #####
    def update_camera_frame(self):
        ret, frame = self.cap.read()
        if ret:
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.usbcam.setPixmap(pixmap)

    ##### 버튼 클릭 이벤트 처리 #####
    def handle_job1(self):
        message = "Job1: red*2, blue*1, goto goal 1"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def handle_job2(self):
        message = "Job2: red*1, blue*2, goto goal 2"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def handle_job3(self):
        message = "Job3: red*1, goto goal 3"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def handle_capture(self):
        ret, frame = self.cap.read()
        if ret:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.png"
            cv2.imwrite(filename, frame)
            self.log_display.append(f"{current_time}화면을 캡쳐했습니다.{filename}")

    def handle_start(self):
        message = "start"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def handle_stop(self):
        message = "stop"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def handle_resume(self):
        message = "resume"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def handle_pause(self):
        message = "pause"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def handle_reset(self):
        message = "reset"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def handle_train(self):
        message = "train"
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    ##### 이메일 발송 #####
    def send_email(self):
        recipient = self.recipient.text().strip()
        '''if not recipient:
            QtWidgets.QMessageBox.warning(None, "Error", "Please enter a valid email address.")
            return'''

        # Email settings
        smtp_server = "smtp.gmail.com"
        smtp_port = 587
        sender_email = "a12012032@gmail.com"
        sender_password = "xyoz mbkb fcdr lnrm"

        # Email content
        subject = "[E-2] 오류 발생"
        body = f"작업 중 오류가 발생하였습니다 확인 부탁드립니다.\n오류 발생 시간: {self.elapsed_time}초"
        msg = MIMEText(body, _charset="utf-8")
        msg["Subject"] = subject
        msg["From"] = sender_email
        msg["To"] = recipient

        try:
            server = smtplib.SMTP(smtp_server, smtp_port)
            server.starttls()
            server.login(sender_email, sender_password)
            server.sendmail(sender_email, recipient, msg.as_string())
            server.quit()
            QtWidgets.QMessageBox.information(None, "Success", f"이메일 전송 완료: {recipient}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(None, "Error", f"이메일 전송 불가: {str(e)}")

    ##### 종료 처리 #####
    def closeEvent(self, event):
        self.camera_timer.stop()
        self.cap.release()
        super().closeEvent(event)


##### 메인 실행 #####
def main():
    # ROS 2 초기화
    rclpy.init()

    # ROS 2 노드 생성
    ros_node = ROSPublisherNode()

    # PyQt 애플리케이션 실행
    app = QtWidgets.QApplication(sys.argv)

    # 로그인 창 표시
    login_dialog = LoginDialog()
    if login_dialog.exec_() == QDialog.Accepted:  # 로그인 성공 시
        # 메인 애플리케이션 실행
        window = SimpleApp(ros_node)
        window.show()

        # Qt 애플리케이션과 ROS 2 스핀 통합
        timer = QtCore.QTimer()
        timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.1))
        timer.start(100)

        sys.exit(app.exec_())

    # ROS 2 종료
    rclpy.shutdown()


if __name__ == '__main__':
    main()
