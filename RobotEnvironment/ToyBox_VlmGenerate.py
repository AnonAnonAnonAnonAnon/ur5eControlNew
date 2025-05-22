from robot_environment import RobotEnvironment
from photo import photo_shot
import time

photo_shot()
robot_environment = RobotEnvironment(pc_id=2)

toy_x, toy_y = robot_environment.get_obj_position_in_robot('toy')
print(f"玩具位置: {toy_x}, {toy_y}")
box_x, box_y = robot_environment.get_obj_position_in_robot('box')
print(f"箱子位置: {box_x}, {box_y}")
time.sleep(2)
robot_environment.move_to_position_in_robot(toy_x, toy_y, 0)
time.sleep(2)
robot_environment.gripper_grasp()
time.sleep(2)
robot_environment.move_to_position_in_robot((toy_x + box_x) / 2, (toy_y + box_y) / 2, 2)
time.sleep(2)
robot_environment.move_to_position_in_robot(box_x, box_y, 2)
time.sleep(2)
robot_environment.gripper_open()
time.sleep(2)