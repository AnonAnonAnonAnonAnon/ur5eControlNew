#最简单DEMO
#polyscope 切换到远程控制
#环境：ur5econtrol
#注意夹爪接线

import serial,time,keyboard
import socket

def main():
    mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mySocket.settimeout(10)

    # change the robot IP address here

    #左机
    # host = '192.168.0.201'
    #右机
    host = '192.168.1.15'

    port = 30001
    mySocket.connect((host, port))
    # print(mySocket.recv(4096).decode())

    # 夹爪
    # 取决于具体串口 
    # ls /dev
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    # ser = serial.Serial('/dev/ttyCH341USB0',9600,timeout=1)
    # ser = serial.Serial('/dev/ttyCH341USB1',9600,timeout=1)

    time.sleep(2)
    motor_open_list = (0x02,0x00,0x20,0x2f,0x00,0,0xa4) #机械爪松开(具体解释见机械爪用户手册)
    motor_close_list = (0x02,0x01,0x20,0x2f,0x00,0,0xa4)    #机械爪闭合，45字节是角度

    while True:
        if keyboard.is_pressed('esc'):
            print('退出程序')
            break
        if keyboard.is_pressed('b'):
            print('开始执行程序！')
            #目标上方,使用发送URSCRIPT代码的方式控制机械臂运动t
            mySocket.send("movel(p[-0.077, -0.636, 0.341, 2.778, -0.994, 0.047], a=1.2, v=0.2)\n".encode())
            time.sleep(4.5)
            #目标位置
            mySocket.send("movel(p[-0.091, -0.678, 0.214, 2.943, -1.057, 0.046], a=1.2, v=0.2)\n".encode())
            time.sleep(1.2)
            ser.write(motor_close_list)     #抓取
            time.sleep(2)
            #目标上方
            mySocket.send("movel(p[-0.077, -0.636, 0.341, 2.778, -0.994, 0.047], a=1.2, v=0.2)\n".encode())
            time.sleep(1.2)
            #放置上方
            mySocket.send("movel(p[-0.553, -0.0188, 0.36397, 1.266, -2.572, -0.049], a=1.2, v=0.2)\n".encode())
            time.sleep(4.5)
            #放置位置
            mySocket.send("movel(p[-0.524, -0.019, 0.211, 1.208, -2.883, -0.001], a=1.2, v=0.2)\n".encode())
            time.sleep(1.5)
            ser.write(motor_open_list)  #放开
        time.sleep(1)
    

    # 关闭串口和网络连接
    ser.close()
    mySocket.close()

if __name__ == '__main__':
    main()