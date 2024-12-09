import socket
from machine import PWM, Pin
import network
import time
import ujson
import _thread

# ######################################################
# 开始设置全局变量
# ######################################################
SSID = 'Yi'
PASSWORD = '88889999'
IFCONFIG = ("192.168.3.190", "255.255.255.0", "192.168.3.1", "8.8.8.8")
PORT_DATA = 12345
PORT_PID = 12346

# PID 控制参数
Kp_dist = 0.2
Ki_dist = 0.001
Kd_dist = 0.08

Kp_angle = 4
Ki_angle = 0.03
Kd_angle = 1

motor_min_duty = 120

# 初始化误差
previous_error_dist = 0
integral_dist = 0

previous_error_angle = 0
integral_angle = 0

# 完成任务条件
SUCCESS_ANGLE = 10
SUCCESS_DISTANCE = 100

# 设置电机控制引脚
IN1 = 12
IN2 = 14
IN3 = 27
IN4 = 26
# ######################################################
# 结束设置全局变量
# ######################################################

left_pwm1 = PWM(Pin(IN4), freq=1000)
left_pwm2 = PWM(Pin(IN3), freq=1000)
right_pwm1 = PWM(Pin(IN2), freq=1000)
right_pwm2 = PWM(Pin(IN1), freq=1000)

# 连wifi
wlan = network.WLAN(network.STA_IF)
wlan.ifconfig(IFCONFIG)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

print('Connecting to WiFi...', end='')
while not wlan.isconnected():
    time.sleep(0.5)
    print('.', end='')
print('\nConnection successful')
print(wlan.ifconfig())


def apply_motor_min_duty(speed):
    if abs(speed) < motor_min_duty:
        return 0
    return speed

def stop_motor():
    global previous_error_dist, integral_dist, previous_error_angle, integral_angle
    left_pwm1.duty(0)
    left_pwm2.duty(0)
    right_pwm1.duty(0)
    right_pwm2.duty(0)
    previous_error_dist = 0
    integral_dist = 0
    previous_error_angle = 0
    integral_angle = 0
    print("stop_motor!")


# 函数：设置电机速度
def set_motor_speed(left_speed, right_speed):
    if left_speed > 0:
        left_speed += motor_min_duty
    else:
        left_speed -= motor_min_duty
    if right_speed > 0:
        right_speed += motor_min_duty
    else:
        right_speed -= motor_min_duty

    # left_speed = apply_motor_min_duty(left_speed)
    # right_speed = apply_motor_min_duty(right_speed)

    left_speed = max(-1023, min(1023, left_speed))
    right_speed = max(-1023, min(1023, right_speed))

    if left_speed < 0:
        left_pwm1.duty(0)
        left_pwm2.duty(abs(left_speed))
    else:
        left_pwm1.duty(left_speed)
        left_pwm2.duty(0)

    if right_speed < 0:
        right_pwm1.duty(0)
        right_pwm2.duty(abs(right_speed))
    else:
        right_pwm1.duty(right_speed)
        right_pwm2.duty(0)


def pid_control(angle, distance):
    global previous_error_dist, integral_dist, previous_error_angle, integral_angle

    # 计算距离误差
    error_dist = distance
    integral_dist += error_dist
    derivative_dist = error_dist - previous_error_dist
    previous_error_dist = error_dist

    # 计算角度误差
    error_angle = angle
    integral_angle += error_angle
    derivative_angle = error_angle - previous_error_angle
    previous_error_angle = error_angle

    # PID 控制计算
    control_signal_dist = Kp_dist * error_dist + Ki_dist * integral_dist + Kd_dist * derivative_dist
    control_signal_angle = Kp_angle * error_angle + Ki_angle * integral_angle + Kd_angle * derivative_angle

    # 计算左右轮速度
    left_speed = int(control_signal_dist + control_signal_angle)
    right_speed = int(control_signal_dist - control_signal_angle)

    return left_speed, right_speed


def handle_data_connection(conn_data):
    while True:
        data = conn_data.recv(1024)
        if not data:
            break
        try:
            command = data.decode().strip()

            # 没有检测到目标, 暂停电机
            if command == "0:0":
                stop_motor()
                conn_data.sendall(b'ACK')
                continue

            # 解析角度和距离数据
            angle, distance = map(float, command.split(':'))
            print(f"Received angle: {angle}, distance: {distance}")

            # 判断如果完成任务, 暂停电机
            if abs(angle) < SUCCESS_ANGLE and abs(distance) < SUCCESS_DISTANCE:
                stop_motor()
                conn_data.sendall(b'ACK')
                continue

            # 利用角度和距离数据, PID方式控制电机
            left_speed, right_speed = pid_control(angle, distance)
            print(f"Left speed: {left_speed}, Right speed: {right_speed}")
            set_motor_speed(left_speed, right_speed)

            # 给客户端返回确认消息, 让客户端进入下一个循环, 不断发送新数据
            conn_data.sendall(b'ACK')

        except Exception as e:
            print(f"Error: {e}")
            stop_motor()
            conn_data.sendall(b'ACK')

    conn_data.close()


def handle_pid_connection(conn_pid):
    global Kp_dist, Ki_dist, Kd_dist, Kp_angle, Ki_angle, Kd_angle
    data = conn_pid.recv(1024)
    try:
        params = ujson.loads(data.decode().strip())
        if 'Kp_dist' in params:
            Kp_dist = params['Kp_dist']
        if 'Ki_dist' in params:
            Ki_dist = params['Ki_dist']
        if 'Kd_dist' in params:
            Kd_dist = params['Kd_dist']
        if 'Kp_angle' in params:
            Kp_angle = params['Kp_angle']
        if 'Ki_angle' in params:
            Ki_angle = params['Ki_angle']
        if 'Kd_angle' in params:
            Kd_angle = params['Kd_angle']
        print(f"Updated PID parameters:\n{Kp_dist=}\n{Ki_dist=}\n{Kd_dist=}\n{Kp_angle=}\n{Ki_angle=}\n{Kd_angle=}\n")
    except Exception as e:
        print(f"Error: {e}")

    conn_pid.close()


# 创建套接字, 控制电机
sock_data = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_data.bind(('', PORT_DATA))
sock_data.listen(1)
print(f'Socket data listening on port {PORT_DATA}')

# 创建套接字, 设置 pid 参数
sock_pid = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_pid.bind(('', PORT_PID))
sock_pid.listen(1)
print(f'Socket pid listening on port {PORT_PID}')


def start_data_thread():
    while True:
        conn_data, addr_data = sock_data.accept()
        print('Connected by data sender:', addr_data)
        handle_data_connection(conn_data)


def start_pid_thread():
    while True:
        conn_pid, addr_pid = sock_pid.accept()
        print('Connected by PID parameter sender:', addr_pid)
        handle_pid_connection(conn_pid)


# 启动两个线程来处理不同的连接
_thread.start_new_thread(start_data_thread, ())
_thread.start_new_thread(start_pid_thread, ())

if __name__ == '__main__':
    while True:
        time.sleep(1)

