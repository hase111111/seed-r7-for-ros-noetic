#!/usr/bin/env python3

from tork_moveit_tutorial import *
import numpy as np
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory

move_group_name = 'larm_with_waist'

name_list = ['waist_y_joint',
 'waist_p_joint',
 'waist_r_joint',
 'l_shoulder_p_joint',
 'l_shoulder_r_joint',
 'l_shoulder_y_joint',
 'l_elbow_joint',
 'l_elbow_joint_mimic',
 'l_elbow_middle_joint',
 'l_elbow_middle_joint_mimic',
 'l_wrist_y_joint',
 'l_wrist_p_joint',
 'l_wrist_r_joint',
 'l_wrist_to_hand_connector',
 'l_eef_grasp_joint']

theta_opt = [[ 0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
 [ 0.0000,  0.0000,  0.0000, -0.0024, -0.0091, -0.0213, -0.0401, -0.0657,
  -0.0983, -0.1377, -0.1833, -0.2342, -0.2896, -0.3482, -0.4089, -0.4701,
  -0.5308, -0.5894, -0.6448, -0.6957, -0.7413, -0.7807, -0.8133, -0.8389,
  -0.8577, -0.8699, -0.8766, -0.8790, -0.8790, -0.8790],
 [ 1.5700,  1.5700,  1.5700,  1.5669,  1.5559,  1.5358,  1.5051,  1.4630,
   1.4095,  1.3449,  1.2702,  1.1866,  1.0958,  0.9996,  0.9002,  0.7996,
   0.7002,  0.6040,  0.5132,  0.4296,  0.3549,  0.2903,  0.2368,  0.1947,
   0.1640,  0.1439,  0.1329,  0.1290,  0.1290,  0.1290],
 [ 0.0000,  0.0000,  0.0000, -0.0043, -0.0161, -0.0378, -0.0710, -0.1165,
  -0.1744, -0.2443, -0.3251, -0.4155, -0.5137, -0.6176, -0.7251, -0.8339,
  -0.9414, -1.0453, -1.1435, -1.2339, -1.3147, -1.3846, -1.4425, -1.4880,
  -1.5212, -1.5429, -1.5547, -1.5590, -1.5590, -1.5590],
 [ 0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
 [ 0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
 [ 0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000,
   0.0000,  0.0000,  0.0000,  0.0000,  0.0000,  0.0000],
 [ 0.0000,  0.0000,  0.0000, -0.0005, -0.0021, -0.0048, -0.0091, -0.0149,
  -0.0223, -0.0312, -0.0415, -0.0530, -0.0656, -0.0788, -0.0926, -0.1064,
  -0.1202, -0.1334, -0.1460, -0.1575, -0.1678, -0.1767, -0.1841, -0.1899,
  -0.1942, -0.1969, -0.1985, -0.1990, -0.1990, -0.1990]]


mapping = dict()
mapping['waist_y_joint'] = theta_opt[0]
mapping['l_shoulder_p_joint'] = theta_opt[1]
mapping['l_shoulder_r_joint'] = theta_opt[2]
mapping['l_shoulder_y_joint'] = theta_opt[3]
mapping['l_elbow_joint'] = theta_opt[4]
# 'l_elbow_joint'縺ｮ蛟､繧偵☆縺ｹ縺ｦ2蛟阪☆繧・mapping['l_elbow_joint'] = [2 * x for x in mapping['l_elbow_joint']]
mapping['l_elbow_joint_dummy'] = theta_opt[5]
mapping['l_wrist_y_joint'] = theta_opt[6]
mapping['l_wrist_r_joint'] = theta_opt[7]


def escape_right_arm():
    group = MoveGroupCommander('rarm')
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)
    
    # move to initial position
    name = group.get_active_joints()
    for joint in name:
        group.set_joint_value_target(joint, 0.0)
    group.set_joint_value_target('r_shoulder_r_joint', np.pi / 3)
    group.go()

    # move to escape position
    group.set_joint_value_target('r_shoulder_r_joint', 0)
    group.go()

def get_trajectory():
    trajectory = RobotTrajectory()
    trajectory.joint_trajectory.joint_names = name_list
    num_steps = len(theta_opt[0])  # 蛻玲焚・・ 譎る俣繧ｹ繝・ャ繝玲焚・・    
    time_step = 0.1  # 蜷・せ繝・ャ繝励・譎る俣髢馴囈・磯←螳懆ｪｿ謨ｴ・・
    for i in range(num_steps):
        point = JointTrajectoryPoint()

        # 縺吶∋縺ｦ縺ｮ髢｢遽縺ｫ縺､縺・※謖・ｻ､蛟､繧貞叙蠕暦ｼ医↑縺代ｌ縺ｰ0・・        
        joint_positions = []
        for joint in name_list:
            if joint in mapping:
                joint_positions.append(mapping[joint][i])  # 謖・ｮ壹＆繧後◆髢｢遽縺ｮ蛟､
            else:
                joint_positions.append(0.0)  # 謖・ｮ壹′縺ｪ縺・未遽縺ｯ0

        # 蜿門ｾ励＠縺滄未遽隗貞ｺｦ繧定ｨｭ螳・        
        point.positions = joint_positions

        # 譎る俣繧定ｨｭ螳・        
        point.time_from_start = rospy.Duration(time_step * (i + 1))

        # `trajectory` 縺ｫ霑ｽ蜉
        trajectory.joint_trajectory.points.append(point)

    return trajectory

def main():
    group = MoveGroupCommander(move_group_name)
    group.set_max_velocity_scaling_factor(1.0)
    group.set_max_acceleration_scaling_factor(1.0)

    # for initial position
    for joint in name_list:
        if joint in mapping:
            rospy.loginfo(f"joint = {joint}")
            group.set_joint_value_target(joint, mapping[joint][1])  # 謖・ｮ壹＆繧後◆髢｢遽縺ｮ蛟､    
    group.go()
    
    trajectory = get_trajectory()
    group.execute(trajectory, wait=True)
    group.go()


if __name__ == '__main__':
    
    init_node()

    escape_right_arm()

    main()