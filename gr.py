
import serial # for serial communication


ser = serial.Serial(port='/dev/ttyCH341USB0', baudrate=9600, timeout=1)

BYTE_OPEN = 0x00
BYTE_CLOSE = 0x01

AVER_SPEED_10 = (0X00,0XC8) # average speed 10rad/s

MOTOR_OPEN_LIST = (0x02,BYTE_OPEN,0x20,0x43,0x14,*AVER_SPEED_10) # release the gripper
MOTOR_CLOSE_LIST = (0x02,BYTE_CLOSE,0x20,0x43,0x14,*AVER_SPEED_10) # close the gripper


