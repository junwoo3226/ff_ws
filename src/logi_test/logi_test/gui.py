import sys
import cv2
import os
from PyQt5 import QtCore, QtWidgets, uic
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QDialog, QMessageBox, QLineEdit, QFormLayout, QPushButton
from PyQt5.QtCore import QTimer, QTime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import smtplib
from email.mime.text import MIMEText
from datetime import datetime


import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from PyQt5.QtCore import pyqtSignal, QObject, Qt


import cv2.aruco as aruco
import numpy as np
from geometry_msgs.msg import Twist
from logi_test.move2 import Turtlebot3ManipulationTest
import time
current_dir = os.path.dirname(os.path.abspath(__file__))
#ui_path = "/home/juchan/ff_ws/src/logi_test/logi_test/detection.ui"
#pt_path = "/home/juchan/ff_ws/src/logi_test/logi_test/best.pt"
ui_path = os.path.join(current_dir, "detection.ui")
pt_path = os.path.join(current_dir, "best.pt")
cam_idx = 2


##### 로봇과 컨베이어에 토픽 발행하는 노드 #####
class ROSPublisherNode(Node):
    def __init__(self):
        super().__init__('job_publisher')
        self.robot_publisher = self.create_publisher(String, 'robot_topic', 10)
        self.conveyor_publisher = self.create_publisher(String, 'conveyor_topic', 10)
        self.error_publisher = self.create_publisher(String, 'conveyor_error_topic', 10)  # 오류 토픽 퍼블리셔 추가

    def robot_publish_message(self, message):
        msg = String()
        msg.data = message
        self.robot_publisher.publish(msg)
        self.get_logger().info(f"Published: {message}")

    def conveyor_publish_message(self, message):
        msg = String()
        msg.data = message
        self.conveyor_publisher.publish(msg)
        self.get_logger().info(f"Published: {message}")

    def publish_error_message(self, error_message):
        msg = String()
        msg.data = error_message
        self.error_publisher.publish(msg)  # 오류 메시지 발행
        self.get_logger().info(f"Published to conveyor_error_topic: {error_message}")


##### 로봇의 캠에서 이미지를 토픽으로 받고, 객체탐지하는 노드 #####
class ObjectDetectionNode(Node, QObject):
    frame_ready = pyqtSignal(object)

    def __init__(self):
        Node.__init__(self, 'object_detection_node')
        QObject.__init__(self)
        self.subscription = self.create_subscription(
            Image,
            '/rgb_image/compressed_image',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO(current_dir)  # YOLOv8 모델 로드
        self.model.overrides['conf'] = 0.8  # 신뢰도(conf) 0.8 이상으로 설정
        self.get_logger().info("Object Detection Node started.")

    def image_callback(self, msg):
        # ROS2 Image 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv8 객체 탐지 수행
        results = self.model(frame)

        # 탐지된 객체 정보 확인
        detected_objects = results[0].boxes.data  # 탐지된 바운딩 박스 정보
        detected_labels = [self.model.names[int(obj[5])] for obj in detected_objects]

        # 'red' 또는 'blue'가 감지되었는지 확인
        if 'red' in detected_labels or 'blue' in detected_labels:
            annotated_frame = results[0].plot()

        # PyQt Signal로 annotated_frame 전달
        self.frame_ready.emit(annotated_frame)


##### 카메라 보정과 아르코 마커 탐지 기능 #####
class arucoDetector():
    def __init__(self):

        # 카메라 보정을 통해 얻은 카메라 행렬 및 왜곡 계수
        self.camera_matrix = np.array([[908.20256439,   0.0,         368.19809714],
                                       [  0.0,         908.39917552, 302.55233952],
                                       [  0.0,           0.0,           1.0]], dtype=np.float32)

        self.dist_coeffs = np.array([[ 1.03272892e-01, -7.02630586e-01,  7.69369093e-03, -1.91506781e-03,
                                        2.66811555e+00]])

        # ArUco 마커 크기 (단위: 미터)
        self.marker_length = 0.105  # 10cm

        # ArUco Dictionary 및 탐지 파라미터
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
        self.parameters = aruco.DetectorParameters()

    def detect(self, frame):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # ArUco 마커 탐지
        corners, ids, rejected = aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.parameters)

        # 자세와 위치 추정
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

        ret = []
        if ids is not None:

            for i, marker_id in enumerate(ids):
                rvec, tvec = rvecs[i][0], tvecs[i][0]

                # 거리 계산
                distance = np.linalg.norm(tvec)
                ret.append(
                    {"id": marker_id[0], "corners": corners, "distance": distance, "tvec": tvec, "rvec": rvec})
        return ret

def calculate_relative_coordinates(rvec1, tvec1, rvec2, tvec2):    # 회전 벡터를 회전 행렬로 변환
    rot_mat1, _ = cv2.Rodrigues(rvec1)
    rot_mat2, _ = cv2.Rodrigues(rvec2)

    # 상대 회전 및 이동 계산
    relative_tvec = rot_mat1.T @ (tvec2 - tvec1)

    ret = []
    for i in range(3):
        ret.append(round(relative_tvec.flatten()[i], 2))
    return ret

def check_stop_condition1(relative_coordinates):    # 상대좌표 조건 확인
    x, y, z = relative_coordinates
    if abs(y) <= 0.5 and z == 0.44:
        return True
    return False

def check_stop_condition2():    # 상대좌표 조건 확인
    x, y, z = [-0.81, -0.15, 0.47]
    if abs(y) <= 0.5 and z == 0.44:
        return True
    return False

def check_grab_position2(relative_coordinates):    # 컨베이어 벨트 뒤쪽 까지
    x, y, z = relative_coordinates
    if abs(x + 0.81) <= 0.05 and abs(y + 0.15) <= 0.05 and abs(z - 0.47) <= 0.05:
        return True
    return False

def check_place_position3_1(relative_coordinates):   # 박스 1번자리에 두는 곳
    x, y, z = relative_coordinates
    if abs(x + 0.09) <= 0.05 and abs(y + 0.18) <= 0.05 and abs(z - 0.4) <= 0.05:
        return True
    return False


##### 로봇에 주행 명령을 보내는 노드 #####
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self):
        msg = Twist()
        msg.linear.x = -0.1  # 앞으로
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("로봇이 이동 중...")

    def move_backward(self):
        msg = Twist()
        msg.linear.x = 0.1  # 뒤로
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("로봇이 이동 중...")

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info("로봇이 정지했습니다.")


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
class MainApp(QtWidgets.QMainWindow):
    def __init__(self, ros_node, detection_node, control_node, aruco_detector):
        super().__init__()
        self.ros_node = ros_node
        self.detection_node = detection_node
        self.control_node = control_node
        self.aruco_detector = aruco_detector
        self.elapsed_time = 0  # 오류 발생 시간

        # UI 파일 불러오기
        uic.loadUi(ui_path, self)

        # UI에서 log, 카메라 QLabel, 버튼, 이메일 입력 필드 찾기
        self.log_display = self.findChild(QtWidgets.QTextEdit, 'log_display')  # QTextBrowser 객체
        self.usbcam = self.findChild(QtWidgets.QLabel, 'usbcam')  # QLabel 객체
        self.job1_button = self.findChild(QtWidgets.QPushButton, 'job1_button')
        self.job2_button = self.findChild(QtWidgets.QPushButton, 'job2_button')
        self.job3_button = self.findChild(QtWidgets.QPushButton, 'job3_button')
        self.recipient = self.findChild(QtWidgets.QLineEdit, 'recipient')  # 이메일 입력 QLineEdit
        self.send_email_button = self.findChild(QtWidgets.QPushButton, 'send_email_button')
        self.conveyor_input = self.findChild(QtWidgets.QLineEdit, 'conveyor_input') # 컨베이너 속도 입력
        self.send_con_num_button = self.findChild(QtWidgets.QPushButton, 'send_con_num_button') # 전송 버튼
        self.turtlecam1 = self.findChild(QtWidgets.QLabel, 'turtlecam1')
        self.turtlecam2 = self.findChild(QtWidgets.QLabel, 'turtlecam2')
        self.current_time_label = self.findChild(QtWidgets.QLabel, 'current_time_label')
        self.operating_time_label = self.findChild(QtWidgets.QLabel, 'operating_time_label')
        self.current_state_display = self.findChild(QtWidgets.QTextEdit, 'current_state_display')

        # UI 상태 준비
        self.log_display.setReadOnly(True)
        self.current_state_display.setReadOnly(True)
        self.update_time()

        # 타이머로 매초 업데이트
        timer = QTimer(self)
        timer.timeout.connect(self.update_time)
        timer.start(1000)  # 1초마다 업데이트

        # UI 로드 확인
        if not all([self.log_display, self.usbcam, self.job1_button, self.job2_button,
                    self.job3_button, self.recipient, self.send_email_button]):
            print("Error: Required widgets not found in the UI file.")
            sys.exit()


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

        self.oper_timer = QTimer(self)
        self.oper_timer.timeout.connect(self.operating_time)  # 타이머 이벤트 핸들러

        self.running = False  # 타이머 동작 상태

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

        # ObjectDetectionNode의 Signal 연결
        self.detection_node.frame_ready.connect(self.update_turtle_cams)
        
        self.send_con_num_button.clicked.connect(self.send_con_num)

        # 이미지 업데이트 순서 관리
        self.frame_counter = 0

        self.current_state_display.setText("대기중")

    ##### 카메라 프레임 업데이트 #####
    def update_camera_frame(self):
        ret, frame = self.cap.read()
        markers = self.aruco_detector.detect(frame)
        if ret:
            # ArUco 마커 데이터 처리
            for item in markers:
                tvec = item["tvec"]
                rvec = item["rvec"]
                corners = item["corners"]

                cv2.drawFrameAxes(frame, self.aruco_detector.camera_matrix,
                                self.aruco_detector.dist_coeffs, rvec, tvec, self.aruco_detector.marker_length)

                aruco.drawDetectedMarkers(frame, corners)

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.usbcam.setPixmap(pixmap)

    def update_turtle_cams(self, frame):
        # OpenCV 이미지를 PyQt QPixmap으로 변환
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_frame.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(320, 240, Qt.KeepAspectRatio, Qt.SmoothTransformation)    # 320x240으로 이미지 리스케일링

        # frame_counter에 따라 이미지를 번갈아가며 표시
        if self.frame_counter % 2 == 0:
            self.turtlecam1.setPixmap(scaled_pixmap)
        else:
            self.turtlecam2.setPixmap(scaled_pixmap)

        self.frame_counter += 1

    ##### 버튼 클릭 이벤트 처리 #####
    def handle_job1(self):
        message = "Job1: red*2, blue*1, goto goal 1"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")

        # 메시지 출력 및 타이머 시작
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")
        self.time1 = QTime.currentTime()
        self.start_timer()
        self.current_state_display.setText("팔레트로 이동중")
        self.aruco_move()

        message = f"move 10000"  # 명령어 포맷 (예: move 100)
        self.ros_node.conveyor_publish_message(message)
        
        self.current_state_display.setText("박스 옮기는 중")
        # Turtlebot3ManipulationTest 인스턴스 생성 및 시나리오 실행
        node = Turtlebot3ManipulationTest()
        #1번위치
        node.send_gripper_goal(0.025)
        node.initialpose()
        time.sleep(3)
        node.ex1_1()
        time.sleep(5)
        node.send_gripper_goal(-0.015)
        time.sleep(1)
        node.initialpose()
        time.sleep(5)
        node.ex6_3()
        time.sleep(6)
        node.send_gripper_goal(0.025)
        node.initialpose()
        
        # #3번위치
        node.send_gripper_goal(0.025)
        node.initialpose()
        time.sleep(3)
        node.ex3_1()
        time.sleep(5)
        node.send_gripper_goal(-0.015)
        time.sleep(1)
        node.initialpose()
        time.sleep(5)
        node.ex6_3()
        time.sleep(6)
        node.send_gripper_goal(0.025)
        node.initialpose()
        
        #4번위치
        node.send_gripper_goal(0.025)
        node.initialpose()
        time.sleep(3)
        node.ex4_1()
        time.sleep(5)
        node.send_gripper_goal(-0.015)
        time.sleep(1)
        node.initialpose()
        time.sleep(5)
        node.ex6_3()
        time.sleep(6)
        node.send_gripper_goal(0.025)
        node.initialpose()
        
        #6번위치
        # node.send_gripper_goal(0.025)
        # node.initialpose()
        # time.sleep(3)
        # node.ex6_1()
        # time.sleep(5)
        # node.send_gripper_goal(-0.015)
        # time.sleep(1)
        # node.initialpose()
        # time.sleep(5)
        # node.ex6_3()
        # time.sleep(6)
        # node.send_gripper_goal(0.025)
        # node.initialpose()

        # self.aruco_move2()
        # self.aruco_move3()

        self.aruco_move_back()

    def handle_job2(self):
        message = "Job2: red*1, blue*2, goto goal 2"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")
        self.time1 = QTime.currentTime()
        self.start_timer()
        self.aruco_move()

    def handle_job3(self):
        message = "Job3: red*1, goto goal 3"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")
        self.time1 = QTime.currentTime()
        self.start_timer()
        self.aruco_move()

    def handle_capture(self):
        ret, frame = self.cap.read()
        if ret:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            current_time = datetime.now().strftime("[%H:%M:%S] - ")
            filename = f"capture_{timestamp}.png"
            cv2.imwrite(filename, frame)
            self.log_display.append(f"{current_time}화면을 캡쳐했습니다.{filename}")

    def handle_start(self):
        message = "start"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")
        self.time1 = QTime.currentTime()
        self.start_timer()

    def handle_stop(self):
        message = "stop"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")
        self.stop_timer()

    def handle_resume(self):
        message = "resume"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")
        self.start_timer()

    def handle_pause(self):
        message = "pause"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")
        self.stop_timer()

    def handle_reset(self):
        message = "reset"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")
        self.stop_timer()

    def handle_train(self):
        message = "train"
        current_time = datetime.now().strftime("[%H:%M:%S] - ")
        self.ros_node.robot_publish_message(message)
        self.log_display.append(f"{current_time}{message}")

    def update_time(self):
        # 현재 시간 가져와서 QLabel에 설정
        current_time = QTime.currentTime().toString("hh:mm:ss")
        self.current_time_label.setText(current_time)

    def start_timer(self):
        self.running = True
        self.oper_timer.start(1000)  # 1초마다 operating_time 호출
    
    def stop_timer(self):
        self.running = False
        self.oper_timer.stop()  # 타이머 정지

    def operating_time(self):
        if self.running:
            time2 = QTime.currentTime()
            operating_time = self.time1.secsTo(time2)
            self.operating_time_label.setText(f"{operating_time}초")

    ##### 이메일 발송 #####
    def send_email(self):
        recipient = self.recipient.text().strip()

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

        
    #### 아르코 마커로 이동 ####
    def aruco_move(self):
        while True:
            ret, frame = self.cap.read()
            markers = self.aruco_detector.detect(frame)
            rvecs = [[], []]
            tvecs = [[], []]

            if ret:
                for item in markers:
                    id = item["id"]
                    tvec = item["tvec"]
                    rvec = item["rvec"]

                    if id == 0:
                        rvecs[0] = rvec
                        tvecs[0] = tvec
                    elif id == 16:
                        rvecs[1] = rvec
                        tvecs[1] = tvec

                if len(rvecs) == 2 and len(rvecs[0]) != len([]) and len(rvecs[1]) != len([]):
                    relative_coordinates = calculate_relative_coordinates(rvecs[0], tvecs[0], rvecs[1], tvecs[1])
                    print(f"상대좌표(id0 원점, id16 원점): {relative_coordinates}")

                    # 정지 조건 확인
                    if check_stop_condition1(relative_coordinates):
                        self.control_node.stop()
                        break
                    else:
                        self.control_node.move_forward()

            else:
                print("마커를 찾을 수 없습니다.")


    #### 아르코 마커로 이동 ####
    def aruco_move_back(self):
        while True:
            ret, frame = self.cap.read()
            markers = self.aruco_detector.detect(frame)
            rvecs = [[], []]
            tvecs = [[], []]

            if ret:
                for item in markers:
                    id = item["id"]
                    tvec = item["tvec"]
                    rvec = item["rvec"]

                    if id == 32:
                        rvecs[0] = rvec
                        tvecs[0] = tvec
                    elif id == 16:
                        rvecs[1] = rvec
                        tvecs[1] = tvec

                if len(rvecs) == 2 and len(rvecs[0]) != len([]) and len(rvecs[1]) != len([]):
                    relative_coordinates = calculate_relative_coordinates(rvecs[0], tvecs[0], rvecs[1], tvecs[1])
                    print(f"상대좌표(id32 원점, id16 원점): {relative_coordinates}")

                    # 정지 조건 확인
                    if check_stop_condition2():
                        self.control_node.stop()
                        break
                    else:
                        self.control_node.move_backward()

            else:
                print("마커를 찾을 수 없습니다.")
                

    ### 컨베이너에 시리얼 통신으로 사용자가 숫자를 입력하면 그 수만큼 클럭 컨베이너 이동 ###
    def send_con_num(self):
        try:
            # 입력된 숫자 가져오기
            conveyor_input = int(self.conveyor_input.text())
            if conveyor_input <= 0:
                raise ValueError("Input must be a positive number.")
            
            current_time = datetime.now().strftime("[%H:%M:%S] - ")

            # ROS 2를 통해 메시지 퍼블리싱
            message = f"move {conveyor_input}"  # 명령어 포맷 (예: move 100)
            self.ros_node.conveyor_publish_message(message)
            self.log_display.append(f"{current_time}컨베이어를 {conveyor_input} 스텝 이동합니다.")
        except ValueError as e:
            QtWidgets.QMessageBox.warning(self, "Invalid Input", "양의 정수를 입력해주세요.")

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
    node = Turtlebot3ManipulationTest()
    ros_node = ROSPublisherNode()
    object_detection_node = ObjectDetectionNode()
    robot_control_node = RobotController()
    aruco_detector = arucoDetector()

    # PyQt 애플리케이션 실행
    app = QtWidgets.QApplication(sys.argv)

    # 로그인 창 표시
    login_dialog = LoginDialog()
    if login_dialog.exec_() == QDialog.Accepted:  # 로그인 성공 시
        # 메인 애플리케이션 실행
        window = MainApp(ros_node, object_detection_node, robot_control_node, aruco_detector)
        window.show()

        # Qt 애플리케이션과 ROS 2 스핀 통합
        timer = QtCore.QTimer()
        timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.1))
        timer.start(100)

        # 두 번째 노드 (object_detection_node)를 위한 타이머
        detection_timer = QtCore.QTimer()
        detection_timer.timeout.connect(lambda: rclpy.spin_once(object_detection_node, timeout_sec=0.1))
        detection_timer.start(100)

        sys.exit(app.exec_())
    node.initialpose()
    # ROS 2 종료
    rclpy.shutdown()


if __name__ == '__main__':
    main()
