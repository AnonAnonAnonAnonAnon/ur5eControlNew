
#导入环境包
from robot_environment import RobotEnvironment

#初始化机器人对象
robot_environment = RobotEnvironment(pc_id=2)

# 可视化界面，键盘控制移动
# robot_environment.run_loop()

# 电影1：抓一个玩具放到盒子上
# 参数：玩具x,y，盒子x,y
robot_environment.script_grasp_one_toy_on_box(-0.66,-0.21,-0.56,-0.51)

