import json
import socket

ESP32_IP = '192.168.3.190'  # 修改为你的 ESP32 的 IP 地址
ESP32_PORT = 12346  # 修改为你的 ESP32 上的端口号

# PID 控制参数
para_dict = dict(
    Kp_dist=0.2,
    Ki_dist=0.001,
    Kd_dist=0.08,

    Kp_angle=4,
    Ki_angle=0.03,
    Kd_angle=1,
)
json_str = json.dumps(para_dict) + '\n'
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((ESP32_IP, ESP32_PORT))
sock.sendall(json_str.encode())
sock.close()

# # PID 控制参数
# Kp_dist = 0.2
# Ki_dist = 0.001
# Kd_dist = 0.08
#
# Kp_angle = 4
# Ki_angle = 0.02
# Kd_angle = 1
