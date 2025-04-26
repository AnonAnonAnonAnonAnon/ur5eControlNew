
#导入环境包
from robot_environment import RobotEnvironment
#导入拍照方法
from photo import photo_shot
#导入时间包
import time

#拍照，如果需要
photo_shot()

#初始化机器人对象
robot_environment = RobotEnvironment(pc_id=2)

# warmup
# 可视化界面，键盘控制移动
# robot_environment.run_loop()


# 抓取指定物体到盒子上，视觉检测
# 参数：物体名称，盒子x,y
# robot_environment.Script_Grasp_ToyBox_Vision('toy',-0.6,-0.2)

# 抓取玩具放到笔上
toy_x,toy_y = robot_environment.get_obj_position_in_robot('toy')
print(f"玩具位置: {toy_x}, {toy_y}")
pen_x,pen_y = robot_environment.get_obj_position_in_robot('pen')
print(f"笔位置: {pen_x}, {pen_y}")
time.sleep(2)
robot_environment.move_to_position_in_robot(toy_x,toy_y,0)
time.sleep(2)
robot_environment.gripper_grasp()
time.sleep(2)
robot_environment.move_to_position_in_robot((toy_x+pen_x)/2,(toy_y+pen_y)/2,2)
time.sleep(2)
robot_environment.move_to_position_in_robot(pen_x,pen_y,0)
time.sleep(2)
robot_environment.gripper_open()
time.sleep(2)



