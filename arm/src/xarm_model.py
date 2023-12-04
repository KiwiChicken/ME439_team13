#!/usr/bin/env python3

import uuid, random
import numpy as np
from scipy import interpolate
import rospy


# Create a messed-up model of the arm using a unique ID leading to unique offsets
random.seed(a=uuid.UUID(int=uuid.getnode()))

cmd = np.ndarray((7,2))
deg = np.ndarray((7,2))

#for ii in np.arange(0,6):
#    cmd[ii] = np.array([random.randint(350,650), random.randint(2350,2650)])
#    deg[ii] = np.array([random.randint(-95,-75), random.randint(75,95)])
#rad = np.radians(deg)

deg[0] = np.array([random.randint(-150,-130), random.randint(130,150)])
cmd[0] = np.array([random.randint(150,250), random.randint(750,850)])
deg[1] = np.array([random.randint(-180,-160), random.randint(-10,0)])
cmd[1] = np.array([random.randint(930,970), random.randint(180,220)])
deg[2] = np.array([random.randint(43,55), random.randint(165,180)])
cmd[2] = np.array([random.randint(300,350), random.randint(800,880)])
deg[3] = np.array([random.randint(-112,-100), random.randint(100,112)])
cmd[3] = np.array([random.randint(960,1000), random.randint(0,50)])
deg[4] = np.array([random.randint(-112,-100), random.randint(100,112)])
cmd[4] = np.array([random.randint(0,50), random.randint(960,1000)])
deg[5] = np.array([random.randint(-112,-100), random.randint(100,112)])
cmd[5] = np.array([random.randint(0,50), random.randint(960,1000)])
deg[6] = np.array([random.randint(0,10), random.randint(80,90)])
cmd[6] = np.array([random.randint(90,120), random.randint(580,610)])

rad = np.radians(deg)

#print(cmd, deg)

## Interpolating functions using the model
f_interp_cmd_to_rad_01_model = interpolate.interp1d(cmd[0], rad[0])
f_interp_cmd_to_rad_12_model = interpolate.interp1d(cmd[1], rad[1])
f_interp_cmd_to_rad_23_model = interpolate.interp1d(cmd[2], rad[2])
f_interp_cmd_to_rad_34_model = interpolate.interp1d(cmd[3], rad[3])
f_interp_cmd_to_rad_45_model = interpolate.interp1d(cmd[4], rad[4])
f_interp_cmd_to_rad_56_model = interpolate.interp1d(cmd[5], rad[5])
f_interp_cmd_to_rad_gripper_model = interpolate.interp1d(cmd[6], rad[6])
        
    
# Load parameters from rosparam to keep handy for the functions below: 
# Matched lists of angles and microsecond commands
map_ang_rad_01 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_01')))
map_cmd_01 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_01'))
map_ang_rad_12 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_12')))
map_cmd_12 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_12'))
map_ang_rad_23 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_23')))
map_cmd_23 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_23'))
map_ang_rad_34 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_34')))
map_cmd_34 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_34'))
map_ang_rad_45 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_45')))
map_cmd_45 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_45'))
map_ang_rad_56 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_56')))
map_cmd_56 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_56'))
map_ang_rad_gripper = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_gripper')))
map_cmd_gripper = np.array(rospy.get_param('/servo_cmd_for_mapping_gripper'))

## Interpolating functions using the arm parameters
f_interp_cmd_to_rad_01_params = interpolate.interp1d(map_cmd_01, map_ang_rad_01)
f_interp_cmd_to_rad_12_params = interpolate.interp1d(map_cmd_12, map_ang_rad_12)
f_interp_cmd_to_rad_23_params = interpolate.interp1d(map_cmd_23, map_ang_rad_23)
f_interp_cmd_to_rad_34_params = interpolate.interp1d(map_cmd_34, map_ang_rad_34)
f_interp_cmd_to_rad_45_params = interpolate.interp1d(map_cmd_45, map_ang_rad_45)
f_interp_cmd_to_rad_56_params = interpolate.interp1d(map_cmd_56, map_ang_rad_56)
f_interp_cmd_to_rad_gripper_params = interpolate.interp1d(map_cmd_gripper, map_ang_rad_gripper)


def convert_servo_commands_to_joint_state(cmd_all, arm_is_present):
    cmd00 = cmd_all[0]
    cmd01 = cmd_all[1]
    cmd02 = cmd_all[2]
    cmd03 = cmd_all[3]
    cmd04 = cmd_all[4]
    cmd05 = cmd_all[5]
    cmd06 = cmd_all[6]
    
    # If there is an arm present, just invert the interpolation using the ROS parameters to get back the intended angle.
    if arm_is_present:
        # Interpolate to find joint_state 
        jt00 = f_interp_cmd_to_rad_01_params(cmd00)
        jt01 = f_interp_cmd_to_rad_12_params(cmd01)
        jt02 = f_interp_cmd_to_rad_23_params(cmd02)
        jt03 = f_interp_cmd_to_rad_34_params(cmd03)
        jt04 = f_interp_cmd_to_rad_45_params(cmd04)
        jt05 = f_interp_cmd_to_rad_56_params(cmd05)
        jt06 = f_interp_cmd_to_rad_gripper_params(cmd06)
    else: 
        # Interpolate with the custom arm model: 
        jt00 = f_interp_cmd_to_rad_01_model(cmd00)
        jt01 = f_interp_cmd_to_rad_12_model(cmd01)
        jt02 = f_interp_cmd_to_rad_23_model(cmd02)
        jt03 = f_interp_cmd_to_rad_34_model(cmd03)
        jt04 = f_interp_cmd_to_rad_45_model(cmd04)
        jt05 = f_interp_cmd_to_rad_56_model(cmd05)
        jt06 = f_interp_cmd_to_rad_gripper_model(cmd06)

    
    jt_all = [jt00, jt01, jt02, jt03, jt04, jt05, jt06]
    
    return jt_all