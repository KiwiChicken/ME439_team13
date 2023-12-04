#!/usr/bin/env python3

# ROS node to command a set of HiWonder Bus servos through the "xArm 1S" controller using Sliders. 
# Peter Adamczyk, University of Wisconsin - Madison
# Updated 2023-10-06
 

import tkinter as tk
import time
import traceback 
import rospy
import numpy as np
# IMPORT the messages: 
from sensor_msgs.msg import JointState

# Parameters from the ROS Parameter Server
servo_neutral_cmds_base_to_tip = rospy.get_param('/servo_neutral_cmds_base_to_tip')

map_cmd_01 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_01'))
map_cmd_12 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_12'))
map_cmd_23 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_23'))
map_cmd_34 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_34'))
map_cmd_45 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_45'))
map_cmd_56 = np.array(rospy.get_param('/servo_cmd_for_mapping_joint_56'))
map_cmd_gripper = np.array(rospy.get_param('/servo_cmd_for_mapping_gripper'))

cmd_all = servo_neutral_cmds_base_to_tip  # List of neutral values for each bus servo


# =============================================================================
#   # Publisher for the servo commands. 
# =============================================================================
pub_bus_servo_commands = rospy.Publisher('/servo_commands',JointState,queue_size=1)
bus_servo_commands_msg = JointState()
bus_servo_commands_msg.name = ['cmd00','cmd01','cmd02','cmd03','cmd04','cmd05','cmd06']

    
# Specific functions for the specific Servos/Sliders       
def move_servo_00(cmd):
    global cmd_all, bus_servo_commands_msg
    cmd_all[0] = int(cmd)
    bus_servo_commands_msg.position = cmd_all
    bus_servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(bus_servo_commands_msg)

def move_servo_01(cmd):
    global cmd_all, bus_servo_commands_msg
    cmd_all[1] = int(cmd)
    bus_servo_commands_msg.position = cmd_all
    bus_servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(bus_servo_commands_msg)
    
def move_servo_02(cmd):
    global cmd_all, bus_servo_commands_msg
    cmd_all[2] = int(cmd)
    bus_servo_commands_msg.position = cmd_all
    bus_servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(bus_servo_commands_msg)
    
def move_servo_03(cmd):
    global cmd_all, bus_servo_commands_msg
    cmd_all[3] = int(cmd)
    bus_servo_commands_msg.position = cmd_all
    bus_servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(bus_servo_commands_msg)
    
def move_servo_04(cmd):
    global cmd_all, bus_servo_commands_msg
    cmd_all[4] = int(cmd)
    bus_servo_commands_msg.position = cmd_all
    bus_servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(bus_servo_commands_msg)
    
def move_servo_05(cmd):
    global cmd_all, bus_servo_commands_msg
    cmd_all[5] = int(cmd)
    bus_servo_commands_msg.position = cmd_all
    bus_servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(bus_servo_commands_msg)
    
def move_servo_06(cmd):
    global cmd_all, bus_servo_commands_msg
    cmd_all[6] = int(cmd)
    bus_servo_commands_msg.position = cmd_all
    bus_servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(bus_servo_commands_msg)
    

def shutdown_servos():
#    cmd_all = servo_neutral_cmds_base_to_tip 
#    bus_servo_commands_msg.position = cmd_all
#    bus_servo_commands_msg.header.stamp = rospy.Time.now()
#    pub_bus_servo_commands.publish(bus_servo_commands_msg)    
#    time.sleep(0.2)
    
    # Send the "Sleep" command: a bunch of "Not a Number commands"
    bus_servo_commands_msg.position = [np.NaN]*len(bus_servo_commands_msg.position)
    bus_servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(bus_servo_commands_msg)  

#%% Section to set up a nice Tkinter GUI with sliders. 
def main(): 
    # Initialize ROS node
    rospy.init_node('manual_servo_cmd_sliders', anonymous=False)    
    
    # set up GUI
    root = tk.Tk()
    root.title("Manual Robot Arm Servo Control (Pulses in Microseconds)")
    
    # draw a big slider for servo 0 position
    min_cmd = min(map_cmd_01)
    max_cmd = max(map_cmd_01)
    mid_cmd = servo_neutral_cmds_base_to_tip[0]
    scale0 = tk.Scale(root,
        from_ = 0,
        to = 1000,
        command = move_servo_00,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_00 us: Current calibration valid from {0} to {1}'.format(min_cmd,max_cmd))
    scale0.set(mid_cmd)
    scale0.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 1 position
    min_cmd = min(map_cmd_12)
    max_cmd = max(map_cmd_12)
    mid_cmd = servo_neutral_cmds_base_to_tip[1]
    scale1 = tk.Scale(root,
        from_ = 0,
        to = 1000,
        command = move_servo_01,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_01 us: Current calibration valid from {0} to {1}'.format(min_cmd,max_cmd))
    scale1.set(mid_cmd)
    scale1.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 2 position
    min_cmd = min(map_cmd_23)
    max_cmd = max(map_cmd_23)
    mid_cmd = servo_neutral_cmds_base_to_tip[2]
    scale2 = tk.Scale(root,
        from_ = 0,
        to = 1000,
        command = move_servo_02,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_02 us: Current calibration valid from {0} to {1}'.format(min_cmd,max_cmd))
    scale2.set(mid_cmd)
    scale2.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 3 position
    min_cmd = min(map_cmd_34)
    max_cmd = max(map_cmd_34)
    mid_cmd = servo_neutral_cmds_base_to_tip[3]
    scale3 = tk.Scale(root,
        from_ = 0,
        to = 1000,
        command = move_servo_03,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_03 us: Current calibration valid from {0} to {1}'.format(min_cmd,max_cmd))
    scale3.set(mid_cmd)
    scale3.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 4 position
    min_cmd = min(map_cmd_45)
    max_cmd = max(map_cmd_45)
    mid_cmd = servo_neutral_cmds_base_to_tip[4]
    scale4 = tk.Scale(root,
        from_ = 0,
        to = 1000,
        command = move_servo_04,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_04 us: Current calibration valid from {0} to {1}'.format(min_cmd,max_cmd))
    scale4.set(mid_cmd)
    scale4.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 5 position
    min_cmd = min(map_cmd_56)
    max_cmd = max(map_cmd_56)
    mid_cmd = servo_neutral_cmds_base_to_tip[5]
    scale5 = tk.Scale(root,
        from_ = 0,
        to = 1000,
        command = move_servo_05,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_05 us: Current calibration valid from {0} to {1}'.format(min_cmd,max_cmd))
    scale5.set(mid_cmd)
    scale5.pack(anchor = tk.CENTER)
    
    # draw a big slider for servo 6 position
    min_cmd = min(map_cmd_gripper)
    max_cmd = max(map_cmd_gripper)
    mid_cmd = servo_neutral_cmds_base_to_tip[6]
    scale6 = tk.Scale(root,
        from_ = 0,
        to = 1000,
        command = move_servo_06,
        orient = tk.HORIZONTAL,
        length = 1000,
        label = 'Servo_06 us: Current calibration valid from {0} to {1}'.format(min_cmd,max_cmd))
    scale6.set(mid_cmd)
    scale6.pack(anchor = tk.CENTER)
    

    # run Tk event loop
    root.mainloop()
    
    # Shut the servos down if the window closes 
    shutdown_servos()


if __name__=="__main__":
    try:
        main()

    except:
        traceback.print_exc()
        shutdown_servos()
        pass