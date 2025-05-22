# robot environment

# import necessary libs
import pyrealsense2 as rs # to control the camera
import numpy as np
import os
import pathlib
import time

import logging

import sys
import logging

# import necessary modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
print(sys.path)
# from . import Visualizer
from visualizer import Visualizer
from auto_callibration import AutoCallibrator
from camera_manager import CameraManager
from debug_decorators import debug_decorator, print_debug
from motor_controller import MotorController
from gripper import Gripper
import Servoj_RTDE_UR5.rtde.rtde as rtde
import Servoj_RTDE_UR5.rtde.rtde_config as rtde_config
from Servoj_RTDE_UR5.min_jerk_planner_translation import PathPlanTranslation

#导入gdino
from rekep.perception.gdino import GroundingDINO
#导入photo.py中的photo_shot方法
from photo import photo_shot
#导入cv2
import cv2
#导入main_vision.py中的get_keypoints_from_rekep方法
from main_vision import get_keypoints_from_rekep
# 导入R
from scipy.spatial.transform import Rotation as R
# 导入json
import json



manual_control_time = 0.3 # the time of the manual control trjectory


class RobotEnvironment(Visualizer, MotorController):
    """
    the class to control the robot environment

    Visualizer: to visualize the camera environment
    MotorController: real-time control of the robot
    """
    def __init__(self, pc_id: int = 0):
        """
        initialize the robot environment
        """
        Visualizer.__init__(self, window_name='robot_environment')
        MotorController.__init__(self, pc_id=pc_id)
        if pc_id not in [1, 2]:
            raise ValueError(f'pc_id must be 1 or 2, but got {pc_id}')
        self.pc_id = pc_id
        self.camera_manager = CameraManager(pc_id=pc_id)
        self.camera = self.camera_manager.get_camera('global')

        # MUST WARM UP BEFORE IT MOVES
        self.warmed_up = False

        # robot configs
        self.robot_host = '192.168.1.15' if pc_id == 2 else '192.168.1.201' # the ip address of the robot
        # 注意，这里是机器人的地址，可能会变化。当前正确的机器人ip地址应该在polyscope中查看

        self.robot_port = 30004 # the port of the robot
        self.robot_config_filepath = pathlib.Path(__file__).parent / 'Servoj_RTDE_UR5' / 'control_loop_configuration.xml' # the config file of the robot
        print_debug(f'loading robot config from: {self.robot_config_filepath}', color_name='COLOR_WHITE')
        self.robot_frequency = 500 # the frequency of the robot
        self.alphas = [np.pi/2,0,0,np.pi/2,-np.pi/2,0] # joint angles

        logging.getLogger().setLevel(logging.INFO)
        # get the config of the robot
        conf = rtde_config.ConfigFile(self.robot_config_filepath)
        self.state_names, self.state_types = conf.get_recipe('state')
        self.setp_names, self.setp_types = conf.get_recipe('setp')
        self.watchdog_names, self.watchdog_types = conf.get_recipe('watchdog')
        self.controller = None

        # set up the gripper
        self.gripper = Gripper(self.pc_id)
        self.gripper_work_time = 0.8 # the time of the gripper working

        # set the keys
        self.keys = []
        self.events = []
        self.all_state_ready = False
        self.__set_keys()

        # set properties
        self._add_realtime_properties(
            ['gripper_state',
             'x', 'y', 'z',
             'rx', 'ry', 'rz',
             'tip_direction_x', 'tip_direction_y', 'tip_direction_z',
             'joint_positions_j1', 'joint_positions_j2', 'joint_positions_j3', 
             'joint_positions_j4', 'joint_positions_j5', 'joint_positions_j6',
             'last_grasp_time']
            ,[0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0, time.time()]
        )

    def run_loop(self):
        """
        the main loop of the robot environment
        """
        self.keep_alive = True
        self.state_message_postion = (self.width_left // 7, self.height//10)

        while self.keep_alive:
            # 这里是对相机对象的调用，拍摄RGB图片
            self.set_screen_middle(self.camera.get_color_image())

            # show keys
            self.add_words(words=self.key_message,
                           screen_switch='left',
                           position=self.key_message_postion,
                           color=(255,255,0),
                           thickness=2)

            # 添加：显示当前机械臂的末端旋转，rx, ry, rz，显示到小数点后3位
            # show states
            if self.all_state_ready:
                self.state_message = [f'gripper_state: {self["gripper_state"]}',
                                      f'x, y, z: {self["x"]:.2f}, {self["y"]:.2f}, {self["z"]:.2f}',
                                      f'rx, ry, rz: {self["rx"]:.3f}, {self["ry"]:.3f}, {self["rz"]:.3f}',# 显示末端旋转，rx, ry, rz，显示到小数点后3位
                                      f'tip_x, tip_y, tip_z: {self["tip_direction_x"]:.2f}, {self["tip_direction_y"]:.2f}, {self["tip_direction_z"]:.2f}',
                                      f'j1, j2, j3: {self["joint_positions_j1"]:.2f}, {self["joint_positions_j2"]:.2f}, {self["joint_positions_j3"]:.2f}',
                                      f'j4, j5, j6: {self["joint_positions_j4"]:.2f}, {self["joint_positions_j5"]:.2f}, {self["joint_positions_j6"]:.2f}']
                self.add_words(words=self.state_message,
                            screen_switch='right',
                            position=self.state_message_postion,
                            color=(255,255,0),
                            thickness=2)

            self.show()

    def __set_keys(self,
                   quit_key: str = 'q',
                   warm_up_key: str = 'm',
                   move_forward_key: str = 'w',
                   move_backward_key: str = 's',
                   move_left_key: str = 'a',
                   move_right_key: str = 'd',
                   move_up_key: str = ' ',
                   move_down_key1: str = 'á',
                   move_down_key2: str = 'â',
                   rotate_forward_key: str = 'R',
                   rotate_backward_key: str = 'T',
                   rotate_left_key: str = 'Q',
                   rotate_right_key: str = 'S',
                   rotate_anticlockwise_key: str = '-',
                   rotate_clockwise_key: str = '=',
                   grasp_key: str = ',',
                   open_key: str = '.',
                   ):
        """
        set up the keys
        """
        self.keys.extend([quit_key, warm_up_key, 
                          move_forward_key, move_backward_key,
                          move_left_key, move_right_key,
                          move_up_key, move_down_key1, move_down_key2,
                          rotate_forward_key, rotate_backward_key,
                          rotate_left_key, rotate_right_key,
                          rotate_anticlockwise_key, rotate_clockwise_key,
                          grasp_key, open_key])
        self.events.extend([self.__quit, self.__warm_up, 
                            self.__move_forward, self.__move_backward,
                            self.__move_left, self.__move_right,
                            self.__move_up, self.__move_down, self.__move_down,
                            self.__rotate_forward, self.__rotate_backward,
                            self.__rotate_left, self.__rotate_right,
                            self.__rotate_anticlockwise, self.__rotate_clockwise,
                            self.__grasp, self.__open])

        self.key_message = [f"Press '{quit_key}' to quit",
                            f"Press '{warm_up_key}' to warm up the robot",
                            f"Press '{move_forward_key}' to move forward",
                            f"Press '{move_backward_key}' to move backward",
                            f"Press '{move_left_key}' to move left",
                            f"Press '{move_right_key}' to move right",
                            f"Press SPACE to move up",
                            f"Press SHIFT to move down",
                            f"Press 'UP' to rotate forward",
                            f"Press 'DOWN' to rotate backward",
                            f"Press 'LEFT' to rotate left",
                            f"Press 'RIGHT' to rotate right",
                            f"Press '-' to rotate anticlockwise",
                            f"Press '+' to rotate clockwise",
                            f"Press '<' to grasp",
                            f"Press '>' to open"]
        self.key_message_postion = (self.width_left // 7, self.height//10)


    def __quit(self):
        """
        quit the robot environment
        """
        self.keep_alive = False
        self.close()
        if self.controller is not None:
            self.controller.disconnect()
        sys.exit(0)


    def __warm_up(self, warm_up_time=5):
        """
        warm up the robot
        warm_up_time: period of warming up(default 5 sec)        
        """
        # warm up the robot
        if self.warmed_up:
            return
        self.warmed_up = True
        print_debug('warming up the robot...(please load the program in 5 sec)', color_name='COLOR_WHITE')
        # TODO: warm up the robot

        # connect before warmup
        self.controller = rtde.RTDE(self.robot_host, self.robot_port)
        connection_state = self.controller.connect()
        while connection_state != 0:
            time.sleep(0.5)
            connection_state = self.controller.connect()
        print_debug('Successfully connected to the robot!', color_name='COLOR_YELLOW')

        self.controller.get_controller_version()
        self.controller.send_output_setup(self.state_names, self.state_types, self.robot_frequency)
        self.setp = self.controller.send_input_setup(self.setp_names, self.setp_types)
        self.watchdog = self.controller.send_input_setup(self.watchdog_names, self.watchdog_types)

        self.setp.input_double_register_0 = 0
        self.setp.input_double_register_1 = 0
        self.setp.input_double_register_2 = 0
        self.setp.input_double_register_3 = 0
        self.setp.input_double_register_4 = 0
        self.setp.input_double_register_5 = 0

        self.setp.input_bit_registers0_to_31 = 0

        if not self.controller.send_start():
            sys.exit()

        state = self.controller.receive()
        tcp = state.actual_TCP_pose
        self.watchdog.input_int_register_0 = 2
        self.controller.send(self.watchdog)
        planner = PathPlanTranslation(tcp, tcp, warm_up_time) # the warmup planner
        orientation_const = tcp[3:]

        t_start = time.time()
        while time.time() - t_start < 5:
            state = self.controller.receive()
            t_current = time.time() - t_start

            if state.runtime_state > 1 and t_current <= warm_up_time:
                position_ref, lin_vel_ref, acceleration_ref = planner.trajectory_planning(t_current)
                pose = position_ref.tolist() + orientation_const
                self.list_to_setp(self.setp, pose)
                self.controller.send(self.setp)

        print_debug(f"It took {time.time()-t_start:.2f}s to execute the warm up program")
        
        # then set up the state update loop
        self.ready = True # begin to run the update method

        # end information
        print_debug('robot warmed up!', color_name='COLOR_WHITE')
    
    @staticmethod
    def list_to_setp(setp, list):
        for i in range(6):
            setp.__dict__[f"input_double_register_{i}"] = list[i]
        return setp
    
    def __get_tcp_pose(self)-> list[float]:
        """
        get the tcp pose of the robot
        List[x,y,z,rx,ry,rz] # the pose of the robot
        rx, ry, rz are NOT REAL
        """
        state = self.controller.receive()
        return state.actual_TCP_pose
    
    def __get_joint_positions(self)-> list[float]:
        """
        get the joint positions of the robot
        List[j1,j2,j3,j4,j5,j6] # the positions of the robot
        """
        state = self.controller.receive()
        return state.actual_q

    def __get_tip_direction(self)-> list[float]:
        """
        get the tip direction of the robot
        List[x,y,z] # the direction of the tool tip
        """
        def at2rotmat(alpha,theta):
            """
            get the transformation matrix of the robot
            """
            return np.array([
                [np.cos(theta),-np.sin(theta)*np.cos(alpha),np.sin(theta)*np.sin(alpha)],
                [np.sin(theta),np.cos(theta)*np.cos(alpha),-np.cos(theta)*np.sin(alpha)],
                [0,np.sin(alpha),np.cos(alpha)]
            ])
        six_joints_np = self.__get_joint_positions()
        thetas = [theta for theta in six_joints_np]
        Ts = [
            at2rotmat(alpha, theta) for alpha,theta in zip(self.alphas,thetas)
        ]
        transformation_matrix = np.eye(3)
        for T_mat in Ts:
            transformation_matrix = transformation_matrix@T_mat
        local_vector = np.array([0,0,1])
        local_vector_col = np.array([local_vector]).T # 1D to 2D
        rotated_vector = transformation_matrix@local_vector_col
        rotated_vector = rotated_vector[:,0] # 2D to 1D
        return rotated_vector
    
    def __move_to_pose(self, pose: list[float], trajectory_time: float = 1):
        """
        move the robot to the pose
        input: 
            pose: list[float], [x,y,z,rx,ry,rz] # the pose of the robot
            trajectory_time: float, the time of the trajectory (default 1s)
        """
        if self.ready and self.warmed_up:
            tcp = self.__get_tcp_pose()
            # orientation_const = tcp[3:]
            self.watchdog.input_int_register_0 = 2
            self.controller.send(self.watchdog)
            state = self.controller.receive()

            # path planner
            planner = PathPlanTranslation(tcp, pose, trajectory_time)
            planner_orientation = PathPlanTranslation(tcp[3:]+[0,0,0], pose[3:]+[0,0,0], trajectory_time)

            t_start = time.time()
            while time.time() - t_start < trajectory_time:
                state = self.controller.receive()
                t_current = time.time() - t_start

                if state.runtime_state > 1 and t_current <= trajectory_time:
                    position_ref, lin_vel_ref, acceleration_ref = planner.trajectory_planning(t_current)
                    orientation_ref, lin_vel_ref_orientation, acceleration_ref_orientation = planner_orientation.trajectory_planning(t_current)
                    # pose = position_ref.tolist() + orientation_const
                    pose = position_ref.tolist() + orientation_ref.tolist()
                    self.list_to_setp(self.setp, pose)
                    self.controller.send(self.setp)

    def __move_forward(self, distance: float=0.01, trajectory_time: float = manual_control_time):
        """
        move the robot forward
        input: 
            distance: float, the distance of the movement (default 0.01m)
            trajectory_time: float, the time of the trajectory (default 1s)
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[1] += distance
            self.__move_to_pose(pose, trajectory_time)
    
    def __move_backward(self, distance: float=0.01, trajectory_time: float = manual_control_time):
        """
        move the robot backward
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[1] -= distance
            self.__move_to_pose(pose, trajectory_time)
    
    def __move_left(self, distance: float=0.01, trajectory_time: float = manual_control_time):
        """
        move the robot left
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[0] -= distance
            self.__move_to_pose(pose, trajectory_time)
    
    def __move_right(self, distance: float=0.01, trajectory_time: float = manual_control_time):
        """
        move the robot right
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[0] += distance
            self.__move_to_pose(pose, trajectory_time)
    
    def __move_up(self, distance: float=0.01, trajectory_time: float = manual_control_time):
        """
        move the robot up
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[2] += distance
            self.__move_to_pose(pose, trajectory_time)
    
    def __move_down(self, distance: float=0.01, trajectory_time: float = manual_control_time):
        """
        move the robot down
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[2] -= distance
            self.__move_to_pose(pose, trajectory_time)
    
    def __rotate_forward(self, angle: float=0.1, trajectory_time: float = manual_control_time):
        """
        rotate the robot forward
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[3] += angle # rx+
            self.__move_to_pose(pose, trajectory_time)
    
    def __rotate_backward(self, angle: float=0.1, trajectory_time: float = manual_control_time):
        """
        rotate the robot backward
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[3] -= angle # rx-
            self.__move_to_pose(pose, trajectory_time)

    def __rotate_left(self, angle: float=0.1, trajectory_time: float = manual_control_time):
        """
        rotate the robot left
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[4] += angle # ry+
            self.__move_to_pose(pose, trajectory_time)
    
    def __rotate_right(self, angle: float=0.1, trajectory_time: float = manual_control_time):
        """
        rotate the robot right
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[4] -= angle # ry-
            self.__move_to_pose(pose, trajectory_time)

    def __rotate_anticlockwise(self, angle: float=0.1, trajectory_time: float = manual_control_time):
        """
        rotate the robot anticlockwise
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[5] += angle # rz+
            self.__move_to_pose(pose, trajectory_time)

    def __rotate_clockwise(self, angle: float=0.1, trajectory_time: float = manual_control_time):
        """
        rotate the robot clockwise
        """
        if self.ready and self.warmed_up:   
            pose = self.__get_tcp_pose()
            pose[5] -= angle # rz-
            self.__move_to_pose(pose, trajectory_time)
    
    def __grasp(self):
        """
        grasp the object
        """
        if self.ready and self.warmed_up:
            if time.time()-self['last_grasp_time'] > self.gripper_work_time:
                # self.gripper.grasp()
                self.gripper.close()
                self['last_grasp_time'] = time.time()
            else:
                print_debug(f'Gripper is still working, please wait...', color_name='COLOR_RED')
    def __open(self):
        """
        open the gripper
        """
        if self.ready and self.warmed_up:   
            if time.time()-self['last_grasp_time'] > self.gripper_work_time:
                self.gripper.open()
            else:
                print_debug(f'Gripper is still working, please wait...', color_name='COLOR_RED')
    
    def update(self):
        self['gripper_state'] = self.gripper['gripper_state']
        tcp_pose = self.__get_tcp_pose()
        tip_direction = self.__get_tip_direction()
        joint_positions = self.__get_joint_positions()
        
        self['x'] = tcp_pose[0]
        self['y'] = tcp_pose[1]
        self['z'] = tcp_pose[2]
        self['rx'] = tcp_pose[3]
        self['ry'] = tcp_pose[4]
        self['rz'] = tcp_pose[5]
        self['tip_direction_x'] = tip_direction[0]
        self['tip_direction_y'] = tip_direction[1]
        self['tip_direction_z'] = tip_direction[2]
        self['joint_positions_j1'] = joint_positions[0]
        self['joint_positions_j2'] = joint_positions[1]
        self['joint_positions_j3'] = joint_positions[2]
        self['joint_positions_j4'] = joint_positions[3]
        self['joint_positions_j5'] = joint_positions[4]
        self['joint_positions_j6'] = joint_positions[5]

        self.all_state_ready = True

    def pose7d_world2robot_from_rekep(self,pose7d):

            """
            将[x,y,z,quat]从世界/相机坐标系转换为机器人坐标系
            """
            # vec2mat
            xyz = pose7d[0:3]
            orientation_quat = pose7d[3:]
            mat_pose_world = np.eye(4)
            mat_pose_world[:3,3] = xyz
            mat_pose_world[:3,:3] = R.from_quat(orientation_quat).as_matrix()

            # transformation matrix

            # 绝对路径
            # 从配置文件中获取坐标系变换矩阵
            # with open('/home/ur5/ur5Control/UR5e_Control/RobotEnvironment/auto_callibration.json', 'r') as f:
            with open('/home/liwenbo/project/yt/ur5eControlNew/RobotEnvironment/auto_callibration.json', 'r') as f:
                robot_state = json.load(f)
                # 获取坐标系变换矩阵
                mat_world2robot = np.array(robot_state['misc']['world2robot_homo']) # 从世界坐标系(cam)变换到机器人坐标系(base)

            print(f"DEBUG: mat_world2robot_read_when_transform: {mat_world2robot}")

            transformation_matrix = mat_world2robot

            #转换
            mat_pose_robot = transformation_matrix@mat_pose_world

            # mat2vec
            robot_xyz = mat_pose_robot[:3,3]
            robot_quat = R.from_matrix(mat_pose_robot[:3,:3]).as_quat()
            robot_pose7d = np.hstack((robot_xyz, robot_quat))
            return robot_pose7d

    def script_grasp_one_toy_on_box(self,toy_x,toy_y,box_x,box_y):
        
        #################################################填写
        # 注意，是负值，单位是米
        # 盒子的位置，桌子坐标系
        # 标定点位置： -0.56 -0.51 0.66 -0.21
        target_position_x_1_desk = box_x
        target_position_y_1_desk = box_y
        # 玩具的位置，桌子坐标系
        object_position_x_1_desk = toy_x
        object_position_y_1_desk = toy_y 
        #################################################填写

        # warm up the robot
        self.__warm_up()

        # get current pose
        starting_pose = self.__get_tcp_pose()
        print('starting_pose:', starting_pose)

        #3个高度
        z_middle_1 = 0.26
        z_bottom_1 = 0.20
        z_top_1 = 0.35
        z_toptop = 0.4

        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235

        # 目标放置位置，机器人坐标系
        target_position_x_1 = target_position_x_1_desk + delta_x
        target_position_y_1 = target_position_y_1_desk + delta_y

        # 目标抓取位置 ，机器人坐标系
        object_position_x_1 = object_position_x_1_desk + delta_x
        object_position_y_1 = object_position_y_1_desk + delta_y

        # 目标位姿序列
        # destination poses
        fixed_rotation = starting_pose[3:]
        dest_pose_1_1 = [object_position_x_1, object_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_2 = [object_position_x_1, object_position_y_1, z_bottom_1] + fixed_rotation
        dest_pose_1_3 = [object_position_x_1, object_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_4 = [target_position_x_1, target_position_y_1, z_top_1] + fixed_rotation
        dest_pose_1_5 = [target_position_x_1, target_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_6 = [target_position_x_1, target_position_y_1, z_top_1] + fixed_rotation

        # 控制移动
        # move to the destination poses
        self.__move_to_pose(dest_pose_1_1, 5)
        self.__move_to_pose(dest_pose_1_2, 5)
        self.__grasp()
        self.__move_to_pose(dest_pose_1_3, 5)
        self.__move_to_pose(dest_pose_1_4, 5)
        self.__move_to_pose(dest_pose_1_5, 5)
        self.__open()
        self.__move_to_pose(dest_pose_1_6, 5)

        # 回到初始位置
        # move back to the starting pose
        self.__move_to_pose(starting_pose, 3)

    def script_grasp_six_toy(self):
        # warm up the robot
        self.__warm_up()

        # get current pose
        starting_pose = self.__get_tcp_pose()
        print('starting_pose:', starting_pose)

        #3个高度
        z_middle_1 = 0.26
        z_bottom_1 = 0.20
        z_top_1 = 0.35
        z_toptop = 0.4

        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235
        
        # 6个目标放置位置，桌子坐标系
        target_position_x_1_desk = -0.5
        target_position_y_1_desk = -0.41
        target_position_x_2_desk = -0.5
        target_position_y_2_desk = -0.48
        target_position_x_3_desk = -0.57
        target_position_y_3_desk = -0.41
        target_position_x_4_desk = -0.57
        target_position_y_4_desk = -0.48
        target_position_x_5_desk = -0.64
        target_position_y_5_desk = -0.41
        target_position_x_6_desk = -0.64
        target_position_y_6_desk = -0.48

        # 6个目标放置位置，机器人坐标系
        target_position_x_1 = target_position_x_1_desk + delta_x
        target_position_y_1 = target_position_y_1_desk + delta_y
        target_position_x_2 = target_position_x_2_desk + delta_x
        target_position_y_2 = target_position_y_2_desk + delta_y
        target_position_x_3 = target_position_x_3_desk + delta_x
        target_position_y_3 = target_position_y_3_desk + delta_y
        target_position_x_4 = target_position_x_4_desk + delta_x
        target_position_y_4 = target_position_y_4_desk + delta_y
        target_position_x_5 = target_position_x_5_desk + delta_x
        target_position_y_5 = target_position_y_5_desk + delta_y
        target_position_x_6 = target_position_x_6_desk + delta_x
        target_position_y_6 = target_position_y_6_desk + delta_y

        #################################################填写
        # 6个目标抓取位置，桌子坐标系
        object_position_x_1_desk = -0.54
        object_position_y_1_desk = -0.21
        object_position_x_2_desk = -0.61
        object_position_y_2_desk = -0.08
        object_position_x_3_desk = -0.65
        object_position_y_3_desk = -0.19
        object_position_x_4_desk = -0.70
        object_position_y_4_desk = -0.28
        object_position_x_5_desk = -0.77
        object_position_y_5_desk = -0.14
        object_position_x_6_desk = -0.81
        object_position_y_6_desk = -0.22    
        
        # object position ，机器人坐标系
        object_position_x_1 = object_position_x_1_desk + delta_x
        object_position_y_1 = object_position_y_1_desk + delta_y
        object_position_x_2 = object_position_x_2_desk + delta_x
        object_position_y_2 = object_position_y_2_desk + delta_y
        object_position_x_3 = object_position_x_3_desk + delta_x
        object_position_y_3 = object_position_y_3_desk + delta_y
        object_position_x_4 = object_position_x_4_desk + delta_x
        object_position_y_4 = object_position_y_4_desk + delta_y
        object_position_x_5 = object_position_x_5_desk + delta_x
        object_position_y_5 = object_position_y_5_desk + delta_y
        object_position_x_6 = object_position_x_6_desk + delta_x
        object_position_y_6 = object_position_y_6_desk + delta_y

        # destination poses
        fixed_rotation = starting_pose[3:]
        dest_pose_1_1 = [object_position_x_1, object_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_2 = [object_position_x_1, object_position_y_1, z_bottom_1] + fixed_rotation
        dest_pose_1_3 = [object_position_x_1, object_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_4 = [target_position_x_1, target_position_y_1, z_top_1] + fixed_rotation
        dest_pose_1_5 = [target_position_x_1, target_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_6 = [target_position_x_1, target_position_y_1, z_top_1] + fixed_rotation

        dest_pose_2_1 = [object_position_x_2, object_position_y_2, z_middle_1] + fixed_rotation
        dest_pose_2_2 = [object_position_x_2, object_position_y_2, z_bottom_1] + fixed_rotation
        dest_pose_2_3 = [object_position_x_2, object_position_y_2, z_middle_1] + fixed_rotation
        dest_pose_2_4 = [target_position_x_2, target_position_y_2, z_top_1] + fixed_rotation
        dest_pose_2_5 = [target_position_x_2, target_position_y_2, z_middle_1] + fixed_rotation
        dest_pose_2_6 = [target_position_x_2, target_position_y_2, z_top_1] + fixed_rotation
               
        dest_pose_3_1 = [object_position_x_3, object_position_y_3, z_middle_1] + fixed_rotation
        dest_pose_3_2 = [object_position_x_3, object_position_y_3, z_bottom_1] + fixed_rotation
        dest_pose_3_3 = [object_position_x_3, object_position_y_3, z_middle_1] + fixed_rotation
        dest_pose_3_4 = [target_position_x_3, target_position_y_3, z_top_1] + fixed_rotation
        dest_pose_3_5 = [target_position_x_3, target_position_y_3, z_middle_1] + fixed_rotation
        dest_pose_3_6 = [target_position_x_3, target_position_y_3, z_top_1] + fixed_rotation    

        dest_pose_4_1 = [object_position_x_4, object_position_y_4, z_middle_1] + fixed_rotation
        dest_pose_4_2 = [object_position_x_4, object_position_y_4, z_bottom_1] + fixed_rotation
        dest_pose_4_3 = [object_position_x_4, object_position_y_4, z_middle_1] + fixed_rotation
        dest_pose_4_4 = [target_position_x_4, target_position_y_4, z_top_1] + fixed_rotation
        dest_pose_4_5 = [target_position_x_4, target_position_y_4, z_middle_1] + fixed_rotation
        dest_pose_4_6 = [target_position_x_4, target_position_y_4, z_top_1] + fixed_rotation

        dest_pose_5_1 = [object_position_x_5, object_position_y_5, z_middle_1] + fixed_rotation
        dest_pose_5_2 = [object_position_x_5, object_position_y_5, z_bottom_1] + fixed_rotation
        dest_pose_5_3 = [object_position_x_5, object_position_y_5, z_middle_1] + fixed_rotation
        dest_pose_5_4 = [target_position_x_5, target_position_y_5, z_top_1] + fixed_rotation
        dest_pose_5_5 = [target_position_x_5, target_position_y_5, z_middle_1] + fixed_rotation  
        dest_pose_5_6 = [target_position_x_5, target_position_y_5, z_top_1] + fixed_rotation
        
        dest_pose_6_1 = [object_position_x_6, object_position_y_6, z_middle_1] + fixed_rotation
        dest_pose_6_2 = [object_position_x_6, object_position_y_6, z_bottom_1] + fixed_rotation
        dest_pose_6_3 = [object_position_x_6, object_position_y_6, z_middle_1] + fixed_rotation 
        dest_pose_6_4 = [target_position_x_6, target_position_y_6, z_top_1] + fixed_rotation
        dest_pose_6_5 = [target_position_x_6, target_position_y_6, z_middle_1] + fixed_rotation
        dest_pose_6_6 = [target_position_x_6, target_position_y_6, z_top_1] + fixed_rotation

        # move to the destination poses
        self.__move_to_pose(dest_pose_1_1, 6)
        self.__move_to_pose(dest_pose_1_2, 2)
        self.__grasp()
        self.__move_to_pose(dest_pose_1_3, 6)
        self.__move_to_pose(dest_pose_1_4, 2)
        self.__move_to_pose(dest_pose_1_5, 2)
        self.__open()
        self.__move_to_pose(dest_pose_1_6, 2)


        self.__move_to_pose(dest_pose_2_1, 6)
        self.__move_to_pose(dest_pose_2_2, 2)
        self.__grasp()
        self.__move_to_pose(dest_pose_2_3, 6)
        self.__move_to_pose(dest_pose_2_4, 2)
        self.__move_to_pose(dest_pose_2_5, 2)
        self.__open()
        self.__move_to_pose(dest_pose_2_6, 2)

        self.__move_to_pose(dest_pose_3_1, 6)
        self.__move_to_pose(dest_pose_3_2, 2)
        self.__grasp()
        self.__move_to_pose(dest_pose_3_3, 6)
        self.__move_to_pose(dest_pose_3_4, 2)
        self.__move_to_pose(dest_pose_3_5, 2)
        self.__open()
        self.__move_to_pose(dest_pose_3_6, 2)

        self.__move_to_pose(dest_pose_4_1, 6)
        self.__move_to_pose(dest_pose_4_2, 2)
        self.__grasp()
        self.__move_to_pose(dest_pose_4_3, 6)
        self.__move_to_pose(dest_pose_4_4, 2)
        self.__move_to_pose(dest_pose_4_5, 2)
        self.__open()
        self.__move_to_pose(dest_pose_4_6, 2)

        self.__move_to_pose(dest_pose_5_1, 6)
        self.__move_to_pose(dest_pose_5_2, 2)
        self.__grasp()
        self.__move_to_pose(dest_pose_5_3, 6)
        self.__move_to_pose(dest_pose_5_4, 2)
        self.__move_to_pose(dest_pose_5_5, 2)
        self.__open()
        self.__move_to_pose(dest_pose_5_6, 2)

        self.__move_to_pose(dest_pose_6_1, 6)
        self.__move_to_pose(dest_pose_6_2, 2)
        self.__grasp()
        self.__move_to_pose(dest_pose_6_3, 6)
        self.__move_to_pose(dest_pose_6_4, 2)
        self.__move_to_pose(dest_pose_6_5, 2)
        self.__open()
        self.__move_to_pose(dest_pose_6_6, 2)

        # move back to the starting pose
        self.__move_to_pose(starting_pose, 3)

    def Script_BottleBox_GraspRotatePlace(self,toy_x,toy_y,box_x,box_y,rotate_mode):
        
        ####################### 传入参数 ##########################
        # 注意，是负值，单位是米
        # 盒子的位置，桌子坐标系
        # 标定点位置： -0.56 -0.51 0.66 -0.21
        target_position_x_1_desk = box_x
        target_position_y_1_desk = box_y
        # 玩具的位置，桌子坐标系
        object_position_x_1_desk = toy_x
        object_position_y_1_desk = toy_y 

        ####################### 启动和初始位姿记录 ##########################
        self.__warm_up()
        # 当前位姿
        # get current pose
        starting_pose = self.__get_tcp_pose()
        print('starting_pose:', starting_pose)

        ####################### 常量 ##########################
        #3个高度
        z_middle_1 = 0.26
        z_bottom_1 = 0.20
        z_top_1 = 0.35
        z_toptop = 0.4
        #固定值
        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235
        # 可调整值：爪子长度
        gripper_length = 0.16
        # 可调整值：瓶子的抓握点下的高度
        bottle_half_height = 0.09
        # 可调整值：机械臂不可达的高度
        unreachable_height = 0.24

        # 固定的朝向值
        x_neg = [1.369,-1.033,-1.392]
        y_neg = [1.366,-1.062,0.999]
        x_y_neg = [1.275,-0.900,-0.273]
        x_60_y_30_neg = [1.246,-0.954,0.213]

        # 后退距离
        back_distance = 0.09

        ####################### 坐标换算 ##########################
        # 目标放置位置，机器人坐标系下
        target_position_x_1 = target_position_x_1_desk + delta_x
        target_position_y_1 = target_position_y_1_desk + delta_y

        # 目标抓取位置 ，机器人坐标系下
        object_position_x_1 = object_position_x_1_desk + delta_x
        object_position_y_1 = object_position_y_1_desk + delta_y

        ####################### 旋转 ##########################
        # 初始位姿的旋转
        fixed_rotation = starting_pose[3:]

        # 旋转模式
        # 1:瓶子朝向x- 2:瓶子朝向y- 
        if rotate_mode == 1:
            target_rotation = x_neg
        elif rotate_mode == 2:
            target_rotation = y_neg
        elif rotate_mode == 3:
            target_rotation = x_y_neg
        elif rotate_mode == 4:
            target_rotation = x_60_y_30_neg

        ####################### 爪子长度导致的平面偏置 ##########################
        if rotate_mode == 1:
            target_position_x_1 = target_position_x_1+gripper_length
        elif rotate_mode == 2:
            target_position_y_1 = target_position_y_1+gripper_length
        elif rotate_mode == 3:
            # 除以根号2
            target_position_x_1 = target_position_x_1+gripper_length/np.sqrt(2)
            target_position_y_1 = target_position_y_1+gripper_length/np.sqrt(2)
        elif rotate_mode == 4:
            target_position_x_1 = target_position_x_1+gripper_length/2
            target_position_y_1 = target_position_y_1+gripper_length*np.sqrt(3)/2

        ####################### 后退 ##########################
        # 后退距离
        # 放完瓶子后，后退，避免碰到瓶子
        if rotate_mode == 1:
            target_position_x_1_after_back = target_position_x_1+back_distance
            target_position_y_1_after_back = target_position_y_1
        elif rotate_mode == 2:
            target_position_x_1_after_back = target_position_x_1
            target_position_y_1_after_back = target_position_y_1+back_distance
        elif rotate_mode == 3:
            target_position_x_1_after_back = target_position_x_1+back_distance/np.sqrt(2)
            target_position_y_1_after_back = target_position_y_1+back_distance/np.sqrt(2)
        elif rotate_mode == 4:
            target_position_x_1_after_back = target_position_x_1+back_distance/2
            target_position_y_1_after_back = target_position_y_1+back_distance*np.sqrt(3)/2

        ####################### 动作序列 ##########################
        # destination poses
        dest_pose_1_1 = [object_position_x_1, object_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_2 = [object_position_x_1, object_position_y_1, z_bottom_1] + fixed_rotation
        dest_pose_1_3 = [object_position_x_1, object_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_4 = [target_position_x_1, target_position_y_1, z_top_1] + target_rotation
        dest_pose_1_5 = [target_position_x_1, target_position_y_1, unreachable_height] + target_rotation
        dest_pose_1_6 = [target_position_x_1_after_back, target_position_y_1_after_back, unreachable_height] + target_rotation
        dest_pose_1_7 = [target_position_x_1_after_back, target_position_y_1_after_back, z_top_1] + fixed_rotation

        ####################### 控制移动 ##########################
        # move to the destination poses
        self.__move_to_pose(dest_pose_1_1, 10)
        self.__move_to_pose(dest_pose_1_2, 5)
        self.__grasp()
        self.__move_to_pose(dest_pose_1_3, 5)
        self.__move_to_pose(dest_pose_1_4, 15)
        self.__move_to_pose(dest_pose_1_5, 5)
        self.__open()
        self.__move_to_pose(dest_pose_1_6, 5)
        self.__move_to_pose(dest_pose_1_7, 5)
        # 回到初始位置
        # move back to the starting pose
        self.__move_to_pose(starting_pose, 5)

    def Script_Grasp_ToyBox_Vision(self,obj,box_x,box_y):
        #添加main_vision.py中的gdino提取物体位置功能，提取玩具，在转换机器人坐标系

        ####################### 关键点提取 ##########################
        # data_path = '/home/ur5/ur5Control/UR5e_Control/RobotEnvironment/data'
        data_path = '/home/liwenbo/project/yt/ur5eControlNew/RobotEnvironment/data'

        frame_number = 0
        keypoints = get_keypoints_from_rekep(obj,data_path,frame_number)
        print(f"Debug: Keypoints: {keypoints}")


        ####################### 坐标系转换 ##########################
        # 取第一个关键点的 XYZ
        first_xyz = keypoints[0]
        # 拼接单位四元数 [0,0,0,1]，构成 7D pose
        identity_quat = np.array([0.0, 0.0, 0.0, 1.0])
        pose7d_world = np.hstack((first_xyz, identity_quat)) 
        # 调用对7d位姿的转换方法实现转换
        pose7d_robot = self.pose7d_world2robot_from_rekep(pose7d_world)
        # 机器人坐标系下的xyz
        robot_xyz = pose7d_robot[:3]
        print(f"World XYZ: {first_xyz}")
        print(f"Robot XYZ: {robot_xyz}")
        #提取转换后xyz的x,y
        toy_x_in_robot = robot_xyz[0]
        toy_y_in_robot = robot_xyz[1]
        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235
        # 转换到桌子坐标系
        toy_x_in_desk = toy_x_in_robot - delta_x
        toy_y_in_desk = toy_y_in_robot - delta_y

        ####################### 运动 ##########################
        self.script_grasp_one_toy_on_box(toy_x_in_desk,toy_y_in_desk,box_x,box_y)

    #获取指定物体在桌面坐标系下的位置
    def get_obj_position_in_desk(self,obj):

        ####################### 关键点提取 ##########################
        # data_path = '/home/ur5/ur5Control/UR5e_Control/RobotEnvironment/data'
        data_path = '/home/liwenbo/project/yt/ur5eControlNew/RobotEnvironment/data'

        frame_number = 0
        keypoints = get_keypoints_from_rekep(obj,data_path,frame_number)
        print(f"Debug: Keypoints: {keypoints}")

        ####################### 坐标系转换 ##########################
        # 取第一个关键点的 XYZ
        first_xyz = keypoints[0]
        # 拼接单位四元数 [0,0,0,1]，构成 7D pose
        identity_quat = np.array([0.0, 0.0, 0.0, 1.0])
        pose7d_world = np.hstack((first_xyz, identity_quat)) 
        # 调用对7d位姿的转换方法实现转换
        pose7d_robot = self.pose7d_world2robot_from_rekep(pose7d_world)
        # 机器人坐标系下的xyz
        robot_xyz = pose7d_robot[:3]
        print(f"World XYZ: {first_xyz}")
        print(f"Robot XYZ: {robot_xyz}")
        #提取转换后xyz的x,y
        obj_x_in_robot = robot_xyz[0]
        obj_y_in_robot = robot_xyz[1]
        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235
        # 转换到桌子坐标系
        obj_x_in_desk = obj_x_in_robot - delta_x
        obj_y_in_desk = obj_y_in_robot - delta_y

        return obj_x_in_desk,obj_y_in_desk

    #获取指定物体在机器人坐标系下的位置
    def get_obj_position_in_robot(self,obj):

        ####################### 关键点提取 ##########################
        # data_path = '/home/ur5/ur5Control/UR5e_Control/RobotEnvironment/data'
        data_path = '/home/liwenbo/project/yt/ur5eControlNew/RobotEnvironment/data'

        frame_number = 0
        keypoints = get_keypoints_from_rekep(obj,data_path,frame_number)
        print(f"Debug: Keypoints: {keypoints}")

        ####################### 坐标系转换 ##########################
        # 取第一个关键点的 XYZ
        first_xyz = keypoints[0]
        # 拼接单位四元数 [0,0,0,1]，构成 7D pose
        identity_quat = np.array([0.0, 0.0, 0.0, 1.0])
        pose7d_world = np.hstack((first_xyz, identity_quat)) 
        # 调用对7d位姿的转换方法实现转换
        pose7d_robot = self.pose7d_world2robot_from_rekep(pose7d_world)
        # 机器人坐标系下的xyz
        robot_xyz = pose7d_robot[:3]
        print(f"World XYZ: {first_xyz}")
        print(f"Robot XYZ: {robot_xyz}")
        #提取转换后xyz的x,y
        obj_x_in_robot = robot_xyz[0]
        obj_y_in_robot = robot_xyz[1]

        return obj_x_in_robot,obj_y_in_robot

    #移动到机器人坐标系下指定位置
    #指定高度等级
    def move_to_position_in_robot(self,x,y,z):
  
        # warm up the robot
        self.__warm_up()

        # get current pose
        starting_pose = self.__get_tcp_pose()
        print('starting_pose:', starting_pose)
        fixed_rotation = starting_pose[3:]

        z_bottom_1 = 0.20
        z_middle_1 = 0.26
        z_top_1 = 0.35

        if z == 0:
            z= z_bottom_1
        elif z == 1:
            z= z_middle_1
        elif z == 2:
            z= z_top_1

        # 控制移动
        # move to the destination poses
        self.__move_to_pose([x,y,z]+fixed_rotation, 5)
    
    #移动到桌面坐标系下指定位置
    #指定高度等级:0为贴近桌面，1为距离桌面一个物体的高度，2为机械臂能够安全移动不碰到物体的高度
    def move_to_position_in_desk(self,x,y,z):
  
        # warm up the robot
        self.__warm_up()

        # get current pose
        starting_pose = self.__get_tcp_pose()
        print('starting_pose:', starting_pose)
        fixed_rotation = starting_pose[3:]

        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235

        z_bottom_1 = 0.20
        z_middle_1 = 0.26
        z_top_1 = 0.35

        if z == 0:
            z= z_bottom_1
        elif z == 1:
            z= z_middle_1
        elif z == 2:
            z= z_top_1

        # 控制移动
        # move to the destination poses
        self.__move_to_pose([x+delta_x,y+delta_y,z]+fixed_rotation, 5)

    #抓取指定物体
    def gripper_grasp(self):
        self.__grasp()

    #打开夹爪
    def gripper_open(self):
        self.__open()
    
    def script_Toy_in_Box(self,toy_x,toy_y,box_x,box_y):
        
        #################################################填写
        # 注意，是负值，单位是米
        # 盒子的位置，桌子坐标系
        # 标定点位置： -0.56 -0.51 0.66 -0.21
        target_position_x_1_desk = box_x
        target_position_y_1_desk = box_y
        # 玩具的位置，桌子坐标系
        object_position_x_1_desk = toy_x
        object_position_y_1_desk = toy_y 
        #################################################填写

        # warm up the robot
        self.__warm_up()

        # get current pose
        starting_pose = self.__get_tcp_pose()
        print('starting_pose:', starting_pose)

        #3个高度
        z_middle_1 = 0.26
        z_bottom_1 = 0.19
        z_top_1 = 0.35
        z_toptop = 0.4

        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235

        # 目标放置位置，机器人坐标系
        target_position_x_1 = target_position_x_1_desk + delta_x
        target_position_y_1 = target_position_y_1_desk + delta_y

        # 目标抓取位置 ，机器人坐标系
        object_position_x_1 = object_position_x_1_desk + delta_x
        object_position_y_1 = object_position_y_1_desk + delta_y

        # 目标位姿序列
        # destination poses
        fixed_rotation = starting_pose[3:]
        dest_pose_1_1 = [object_position_x_1, object_position_y_1, z_bottom_1] + fixed_rotation
        dest_pose_1_2 = [(object_position_x_1+target_position_x_1)/2, (object_position_y_1+target_position_y_1)/2, z_top_1] + fixed_rotation
        dest_pose_1_3 = [target_position_x_1, target_position_y_1, z_top_1] + fixed_rotation

        # 控制移动
        # move to the destination poses
        self.__move_to_pose(dest_pose_1_1, 5)
        time.sleep(2)
        self.__grasp()
        time.sleep(2)
        self.__move_to_pose(dest_pose_1_2, 5)
        time.sleep(2)
        self.__move_to_pose(dest_pose_1_3, 5)
        time.sleep(2)
        self.__open()

        # 回到初始位置
        # move back to the starting pose
        self.__move_to_pose(starting_pose, 3)

    
    def script_Cat_in_Box(self,toy_x,toy_y,box_x,box_y):
        
        #################################################填写
        # 注意，是负值，单位是米
        # 盒子的位置，桌子坐标系
        # 标定点位置： -0.56 -0.51 0.66 -0.21
        target_position_x_1_desk = box_x
        target_position_y_1_desk = box_y
        # 玩具的位置，桌子坐标系
        object_position_x_1_desk = toy_x
        object_position_y_1_desk = toy_y 
        #################################################填写

        # warm up the robot
        self.__warm_up()

        # get current pose
        starting_pose = self.__get_tcp_pose()
        print('starting_pose:', starting_pose)

        #3个高度
        z_middle_1 = 0.37
        z_bottom_1 = 0.19
        z_top_1 = 0.45
        z_toptop = 0.4

        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235

        # 目标放置位置，机器人坐标系
        target_position_x_1 = target_position_x_1_desk + delta_x
        target_position_y_1 = target_position_y_1_desk + delta_y

        # 目标抓取位置 ，机器人坐标系
        object_position_x_1 = object_position_x_1_desk + delta_x
        object_position_y_1 = object_position_y_1_desk + delta_y

        # 目标位姿序列
        # destination poses
        fixed_rotation = starting_pose[3:]
        dest_pose_1_0 = [object_position_x_1, object_position_y_1, z_top_1] + fixed_rotation
        dest_pose_1_1 = [object_position_x_1, object_position_y_1, z_bottom_1] + fixed_rotation
        dest_pose_1_2 = [object_position_x_1, object_position_y_1, z_top_1] + fixed_rotation
        dest_pose_1_3 = [(object_position_x_1+target_position_x_1)/2, (object_position_y_1+target_position_y_1)/2, z_top_1] + fixed_rotation
        dest_pose_1_4 = [target_position_x_1, target_position_y_1, z_top_1] + fixed_rotation

        # 控制移动
        # move to the destination poses
        self.__move_to_pose(dest_pose_1_0, 5)
        time.sleep(2)
        self.__move_to_pose(dest_pose_1_1, 5)
        time.sleep(2)
        self.__grasp()
        time.sleep(2)
        self.__move_to_pose(dest_pose_1_2, 5)
        time.sleep(2)
        self.__move_to_pose(dest_pose_1_3, 5)
        time.sleep(2)
        self.__move_to_pose(dest_pose_1_4, 5)
        time.sleep(2)
        self.__open()
        time.sleep(2)

        # 回到初始位置
        # move back to the starting pose
        self.__move_to_pose(starting_pose, 3)


    def Script_PenBox_GraspRotatePlace(self,toy_x,toy_y,box_x,box_y,rotate_mode):
        
        ####################### 传入参数 ##########################
        # 注意，是负值，单位是米
        # 盒子的位置，桌子坐标系
        # 标定点位置： -0.56 -0.51 0.66 -0.21
        target_position_x_1_desk = box_x
        target_position_y_1_desk = box_y
        # 玩具的位置，桌子坐标系
        object_position_x_1_desk = toy_x
        object_position_y_1_desk = toy_y 

        ####################### 启动和初始位姿记录 ##########################
        self.__warm_up()
        # 当前位姿
        # get current pose
        starting_pose = self.__get_tcp_pose()
        print('starting_pose:', starting_pose)

        ####################### 常量 ##########################
        #3个高度
        z_middle_1 = 0.26
        z_bottom_1 = 0.185
        z_top_1 = 0.35
        z_toptop = 0.4
        #固定值
        #机器人坐标系和桌子坐标系之间的偏移
        delta_x = 0.1595
        delta_y = 0.1235
        # 可调整值：爪子长度
        gripper_length = 0.16
        # 可调整值：瓶子的抓握点下的高度
        bottle_half_height = 0.09
        # 可调整值：机械臂不可达的高度
        unreachable_height = 0.24

        # 固定的朝向值
        x_neg = [1.369,-1.033,-1.392]
        y_neg = [1.366,-1.062,0.999]
        x_y_neg = [1.275,-0.900,-0.273]
        x_60_y_30_neg = [1.246,-0.954,0.213]

        # 后退距离
        back_distance = 0.09

        ####################### 坐标换算 ##########################
        # 目标放置位置，机器人坐标系下
        target_position_x_1 = target_position_x_1_desk + delta_x
        target_position_y_1 = target_position_y_1_desk + delta_y

        # 目标抓取位置 ，机器人坐标系下
        object_position_x_1 = object_position_x_1_desk + delta_x
        object_position_y_1 = object_position_y_1_desk + delta_y

        ####################### 旋转 ##########################
        # 初始位姿的旋转
        fixed_rotation = starting_pose[3:]

        # 旋转模式
        # 1:瓶子朝向x- 2:瓶子朝向y- 
        if rotate_mode == 1:
            target_rotation = x_neg
        elif rotate_mode == 2:
            target_rotation = y_neg
        elif rotate_mode == 3:
            target_rotation = x_y_neg
        elif rotate_mode == 4:
            target_rotation = x_60_y_30_neg

        ####################### 爪子长度导致的平面偏置 ##########################
        if rotate_mode == 1:
            target_position_x_1 = target_position_x_1+gripper_length
        elif rotate_mode == 2:
            target_position_y_1 = target_position_y_1+gripper_length
        elif rotate_mode == 3:
            # 除以根号2
            target_position_x_1 = target_position_x_1+gripper_length/np.sqrt(2)
            target_position_y_1 = target_position_y_1+gripper_length/np.sqrt(2)
        elif rotate_mode == 4:
            target_position_x_1 = target_position_x_1+gripper_length/2
            target_position_y_1 = target_position_y_1+gripper_length*np.sqrt(3)/2

        ####################### 后退 ##########################
        # 后退距离
        # 放完瓶子后，后退，避免碰到瓶子
        if rotate_mode == 1:
            target_position_x_1_after_back = target_position_x_1+back_distance
            target_position_y_1_after_back = target_position_y_1
        elif rotate_mode == 2:
            target_position_x_1_after_back = target_position_x_1
            target_position_y_1_after_back = target_position_y_1+back_distance
        elif rotate_mode == 3:
            target_position_x_1_after_back = target_position_x_1+back_distance/np.sqrt(2)
            target_position_y_1_after_back = target_position_y_1+back_distance/np.sqrt(2)
        elif rotate_mode == 4:
            target_position_x_1_after_back = target_position_x_1+back_distance/2
            target_position_y_1_after_back = target_position_y_1+back_distance*np.sqrt(3)/2

        ####################### 动作序列 ##########################
        # destination poses
        dest_pose_1_1 = [object_position_x_1, object_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_2 = [object_position_x_1, object_position_y_1, z_bottom_1] + fixed_rotation
        dest_pose_1_3 = [object_position_x_1, object_position_y_1, z_middle_1] + fixed_rotation
        dest_pose_1_4 = [target_position_x_1, target_position_y_1, z_top_1] + target_rotation
        dest_pose_1_5 = [target_position_x_1, target_position_y_1, unreachable_height] + target_rotation
        dest_pose_1_6 = [target_position_x_1_after_back, target_position_y_1_after_back, unreachable_height] + target_rotation
        dest_pose_1_7 = [target_position_x_1_after_back, target_position_y_1_after_back, z_top_1] + fixed_rotation

        ####################### 控制移动 ##########################
        # move to the destination poses
        self.__move_to_pose(dest_pose_1_1, 10)
        self.__move_to_pose(dest_pose_1_2, 5)
        self.__grasp()
        self.__move_to_pose(dest_pose_1_3, 5)
        self.__move_to_pose(dest_pose_1_4, 20)
        self.__move_to_pose(dest_pose_1_5, 10)
        self.__open()
        # self.__move_to_pose(dest_pose_1_6, 5)
        # self.__move_to_pose(dest_pose_1_7, 10)
        # 回到初始位置
        # move back to the starting pose
        self.__move_to_pose(starting_pose, 5)




if __name__ == '__main__':
    robot_environment = RobotEnvironment(pc_id=2)

    # 可视化界面
    # robot_environment.run_loop()
    # 键盘控制移动

    # 电影
    # robot_environment.script_grasp_one_toy_on_box()
    # robot_environment.script_grasp_six_toy()
    




