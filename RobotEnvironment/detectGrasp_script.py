
#导入环境包
from robot_environment import RobotEnvironment
#导入拍照方法
from photo import photo_shot

#拍照，如果需要
photo_shot()

#初始化机器人对象
robot_environment = RobotEnvironment(pc_id=2)

# warmup
# 可视化界面，键盘控制移动
# robot_environment.run_loop()


# 抓取指定物体到盒子上，视觉检测
# 参数：物体名称，盒子x,y
robot_environment.Script_Grasp_ToyBox_Vision('toy',-0.6,-0.2)


