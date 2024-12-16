import cv2
import cv2.aruco as aruco
import numpy as np


def main():
    # 정확한 카메라 행렬 및 왜곡 계수는 카메라 보정을 통해 얻어야 함
    camera_matrix = np.array([[666.21311517, 0.,    630.12891911],
                              [0.,    663.99283008, 363.5997193],
                              [0.,      0.,       1.]], dtype=np.float32)
    # 보정된 값으로 교체 필요
    dist_coeffs = np.array(
        [-0.03645762, 0.08774861, -0.00096308, -0.00120847, -0.06589077])

    # Aruco 마커 크기 (단위: 미터)
    marker_length = 0.105  # 10cm

    # 카메라 설정
    cap = cv2.VideoCapture(2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return

    # Aruco Dictionary 및 탐지 파라미터
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
    parameters = aruco.DetectorParameters_create()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aruco 마커 탐지
        corners, ids, rejected = aruco.detectMarkers(
            gray, aruco_dict, parameters=parameters)

        if ids is not None:
            # 자세와 위치 추정
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, marker_length, camera_matrix, dist_coeffs)

            for i, marker_id in enumerate(ids):
                rvec, tvec = rvecs[i][0], tvecs[i][0]

                # 거리 계산
                distance = np.linalg.norm(tvec)
                print(f"ID: {marker_id[0]}, 거리: {distance:.2f}m, tvec: {tvec}")

                # 축 그리기
                cv2.drawFrameAxes(frame, camera_matrix,
                                  dist_coeffs, rvec, tvec, marker_length)

            # 마커 표시
            aruco.drawDetectedMarkers(frame, corners, ids)
        else:
            print("마커를 찾을 수 없습니다.")

        cv2.imshow('Aruco Marker Detection', frame)

        # ESC 키로 종료
        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()