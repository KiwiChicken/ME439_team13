#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 10=27=20 2023

@author: pi
"""


# enter an endpoint here and Run the file. The output will be 
ep = [0., 0., 0.2]



###
import InvKinArmRob_serial as IK
import FwdKinArmRob_serial as FK
import numpy as np
from scipy import interpolate

rotational_angles_for_mapping_joint_01= [-90,0,90]
servo_cmd_for_mapping_joint_01= [120, 500, 880]
rotational_angles_for_mapping_joint_12= [-180,-90,0]
servo_cmd_for_mapping_joint_12= [970,600,220]
rotational_angles_for_mapping_joint_23= [0,90,180]
servo_cmd_for_mapping_joint_23= [140,500,880]
rotational_angles_for_mapping_joint_34= [-112,-90,0,90,112]
servo_cmd_for_mapping_joint_34= [1000,890,505,140,0]
rotational_angles_for_mapping_joint_45= [-112,-90,0,90,112]
servo_cmd_for_mapping_joint_45= [0,120,490,880,1000]
rotational_angles_for_mapping_joint_56= [-112,-90,0,90,112]
servo_cmd_for_mapping_joint_56= [0,120,500,880,1000]
rotational_angles_for_mapping_gripper= [0, 90]
servo_cmd_for_mapping_gripper= [90,610]

f_interp_rad_to_cmd_01_params = interpolate.interp1d(rotational_angles_for_mapping_joint_01, servo_cmd_for_mapping_joint_01)
f_interp_rad_to_cmd_12_params = interpolate.interp1d(rotational_angles_for_mapping_joint_12, servo_cmd_for_mapping_joint_12)
f_interp_rad_to_cmd_23_params = interpolate.interp1d(rotational_angles_for_mapping_joint_23, servo_cmd_for_mapping_joint_23)
f_interp_rad_to_cmd_34_params = interpolate.interp1d(rotational_angles_for_mapping_joint_34, servo_cmd_for_mapping_joint_34)
f_interp_rad_to_cmd_45_params = interpolate.interp1d(rotational_angles_for_mapping_joint_45, servo_cmd_for_mapping_joint_45)
f_interp_rad_to_cmd_56_params = interpolate.interp1d(rotational_angles_for_mapping_joint_56, servo_cmd_for_mapping_joint_56)
f_interp_rad_to_cmd_gripper_params = interpolate.interp1d(rotational_angles_for_mapping_gripper, servo_cmd_for_mapping_gripper)



angRad = IK.armrobinvkin(np.array(ep))
angDeg = np.degrees(angRad)
cmd0 = f_interp_rad_to_cmd_01_params(angDeg[0])
cmd1 = f_interp_rad_to_cmd_12_params(angDeg[1])
cmd2 = f_interp_rad_to_cmd_23_params(angDeg[2])
cmd3 = f_interp_rad_to_cmd_34_params(angDeg[3])
cmd4 = f_interp_rad_to_cmd_45_params(angDeg[4])
cmd5 = f_interp_rad_to_cmd_56_params(angDeg[5])
#cmdG = f_interp_rad_to_cmd_gripper_params(angDeg[6])

cmds = np.asfarray([cmd0, cmd1, cmd2, cmd3, cmd4, cmd5])#, cmdG]

print("endpoint: ", ep)
print("angles_deg: ", np.round(angDeg))
print("commands: ", np.round(cmds))


