from robot_environment import RobotEnvironment
from photo import photo_shot
import time

photo_shot()
robot_environment = RobotEnvironment(pc_id=2)

toy_x, toy_y = robot_environment.get_obj_position_in_robot('toy')
pen_x, pen_y = robot_environment.get_obj_position_in_robot('pen')

time.sleep(2)
robot_environment.move_to_position_in_robot(toy_x, toy_y, 2)
time.sleep(2)
robot_environment.move_to_position_in_robot(toy_x, toy_y, 1)
time.sleep(2)
robot_environment.move_to_position_in_robot(toy_x, toy_y, 0)
time.sleep(2)
robot_environment.gripper_grasp()
time.sleep(2)
robot_environment.move_to_position_in_robot(toy_x, toy_y, 2)
time.sleep(2)
robot_environment.move_to_position_in_robot(pen_x, pen_y, 2)
time.sleep(2)
robot_environment.move_to_position_in_robot(pen_x, pen_y, 1)
time.sleep(2)
robot_environment.move_to_position_in_robot(pen_x, pen_y, 0)
time.sleep(2)
robot_environment.gripper_open()
time.sleep(2)
robot_environment.move_to_position_in_robot(pen_x, pen_y, 2)
time.sleep(2)
