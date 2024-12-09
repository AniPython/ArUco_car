from typing import Tuple
from time import sleep
import socket
import cv2
import cv2.aruco as aruco
import numpy as np

# ESP32 的 IP 地址和端口
ESP32_IP = '192.168.3.190'  # 修改为你的 ESP32 的 IP 地址
ESP32_PORT = 12345  # 修改为你的 ESP32 上的端口号

CAR_ID = 10
GOODS_ID = 30

CAMERA_ID = 0


# 计算中点
def calculate_corner_center(corner) -> Tuple[int, int]:
    center_x = np.mean(corner[:, 0])
    center_y = np.mean(corner[:, 1])
    return int(center_x), int(center_y)


# 计算两点的中点
def calculate_2points_center(point1, point2) -> Tuple[int, int]:
    center_x = (point1[0] + point2[0]) / 2
    center_y = (point1[1] + point2[1]) / 2
    return int(center_x), int(center_y)


def draw_lines_by_dots(*points, canvas):
    number_of_points = len(points)

    if number_of_points >= 2:
        cv2.circle(canvas, points[0], 6, (0, 0, 255), -1)

        for i in range(number_of_points - 1):
            cv2.line(canvas, points[i], points[i + 1], (0, 200, 200), 3)
            cv2.circle(canvas, points[i + 1], 6, (0, 0, 255), -1)


def detect_corners_and_send_data():
    # 创建 TCP/IP 套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # 连接到 ESP32
        sock.connect((ESP32_IP, ESP32_PORT))
    except Exception as e:
        print(f"Unable to connect to ESP32: {e}")
        return

    # 打开视频捕捉
    cap = cv2.VideoCapture(CAMERA_ID)  # 0 是默认摄像头索引，如果不行可以尝试 1, 2, 等

    # 检查摄像头是否打开
    if not cap.isOpened():
        print("Cannot open camera")
        return

    # 加载 ArUco 字典
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    while True:
        # 读取摄像头帧
        ret, frame = cap.read()

        if not ret:
            print(f"Can't receive frame from camera {CAMERA_ID}. Exiting ...")
            break

        corners, ids, rejected = detector.detectMarkers(frame)

        # 绘制检测到的标记
        if ids is not None:
            ids_list = list(ids.ravel())
            aruco.drawDetectedMarkers(frame, corners, ids)

            if CAR_ID in ids and GOODS_ID in ids:

                # 获取索引
                corner_car_index = ids_list.index(CAR_ID)
                corner_goods_index = ids_list.index(GOODS_ID)

                # 获取 corner
                corner_car = corners[corner_car_index][0]
                corner_goods = corners[corner_goods_index][0]

                # 获取中心点
                # car 的中心点
                center_car = calculate_corner_center(corner_car)
                # goods 的中心点
                center_goods = calculate_corner_center(corner_goods)
                # car 第一条线(前面)的中心点
                center_car_line1 = calculate_corner_center(corner_car[0:2])

                # 计算向量
                vector1 = np.array(center_car) - np.array(center_car_line1)
                vector2 = np.array(center_car_line1) - np.array(center_goods)

                # 计算向量的角度
                angle = np.arctan2(vector2[1], vector2[0]) - np.arctan2(vector1[1], vector1[0])
                angle = np.degrees(angle)

                # 确保角度在 -180 到 180 之间
                if angle > 180:
                    angle -= 360
                elif angle < -180:
                    angle += 360

                # 计算距离
                distance = np.linalg.norm(vector2)

                # 画多个点的连线
                draw_lines_by_dots(center_car, center_car_line1, center_goods, canvas=frame)

                # 显示文本
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 1
                color = (0, 230, 0)  # 绿色
                thickness = 2
                # 显示当前角度
                cv2.putText(frame, f"Angle: {angle:.2f}", (800, 50),
                            font, font_scale, color, thickness, cv2.LINE_AA)
                # 显示当前距离
                cv2.putText(frame, f"Distance: {distance:.2f}", (800, 100),
                            font, font_scale, color, thickness, cv2.LINE_AA)

                # 数据发到 esp32
                command = f'{angle:.2f}:{distance:.2f}\n'
            else:
                command = f'0:0\n'
        else:
            command = f'0:0\n'

        cv2.imshow('frame', frame)

        sock.sendall(command.encode())
        sock.recv(3)

        # 按 'q' 键退出
        if cv2.waitKey(1) == ord('q'):
            break

    # 释放资源
    cap.release()
    cv2.destroyAllWindows()
    sock.close()


if __name__ == '__main__':
    detect_corners_and_send_data()
