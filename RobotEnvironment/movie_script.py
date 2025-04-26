
#导入环境包
from robot_environment import RobotEnvironment
#导入拍照方法
from photo import photo_shot

#拍照，如果需要
# photo_shot()

#初始化机器人对象
robot_environment = RobotEnvironment(pc_id=2)

# warmup
# 可视化界面，键盘控制移动
# robot_environment.run_loop()

# 电影1：抓一个玩具放到盒子上
# 参数：玩具x,y，盒子x,y
# robot_environment.script_grasp_one_toy_on_box(-0.66,-0.21,-0.56,-0.51)

# 电影2：抓一个玩具，旋转90度，放到盒子上
# 参数：玩具x,y，盒子x,y ; 旋转模式 1:x- 2:y- 3:x-y- 4:x-60y-30
robot_environment.Script_BottleBox_GraspRotatePlace(-0.66,-0.21,-0.67,-0.62,4)

# 电影3：抓取指定物体到盒子上，视觉检测
# 参数：物体名称，盒子x,y
# robot_environment.Script_Grasp_ToyBox_Vision('toy',-0.56,-0.51)


