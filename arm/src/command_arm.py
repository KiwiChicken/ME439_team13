#! /usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
from scipy import interpolate
# IMPORT the messages: 
from sensor_msgs.msg import JointState
# IMPORT the arm controller
import xarm
# IMPORT the custom arm model 
import arm_model 


# Try to set up the PCA9685 PWM chip to command the servos. 
# If it fails, assume there's no arm present and just publish the commands instead. 
try: 
    # arm is the first xArm detected which is connected to USB
    # Note: under the hood it is using "hid" API ("sudo pip3 install hidapi") to get access to the controller. 
    arm = xarm.Controller('USB',debug=True)
    
    arm_is_present = True
    
except: 
    rospy.loginfo('No xArm controller Detected. Publishing only!')
    arm_is_present = False
    

# Load parameters from rosparam to keep handy for the functions below: 
    
# Matched Lists of Servo Indices and ID Numbers
servo_indices_base_to_tip = rospy.get_param('/servo_indices_base_to_tip')
servo_IDs_base_to_tip = rospy.get_param('/servo_IDs_base_to_tip')    
    
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

f_interp_rad_to_cmd_01_params = interpolate.interp1d(map_ang_rad_01, map_cmd_01)
f_interp_rad_to_cmd_12_params = interpolate.interp1d(map_ang_rad_12, map_cmd_12)
f_interp_rad_to_cmd_23_params = interpolate.interp1d(map_ang_rad_23, map_cmd_23)
f_interp_rad_to_cmd_34_params = interpolate.interp1d(map_ang_rad_34, map_cmd_34)
f_interp_rad_to_cmd_45_params = interpolate.interp1d(map_ang_rad_45, map_cmd_45)
f_interp_rad_to_cmd_56_params = interpolate.interp1d(map_ang_rad_56, map_cmd_56)
f_interp_rad_to_cmd_gripper_params = interpolate.interp1d(map_ang_rad_gripper, map_cmd_gripper)



# =============================================================================
#   # Publisher for the servo commands. 
# =============================================================================
pub_bus_servo_commands = rospy.Publisher('/servo_commands',JointState,queue_size=1)
bus_servo_commands_msg = JointState()

# =============================================================================
#   # Publisher for the Joint States. 
# =============================================================================
pub_joint_states = rospy.Publisher('/joint_states',JointState,queue_size=1)
joint_state_msg = JointState()


# =============================================================================
#   # Main function
# =============================================================================
def main(): 
    # =============================================================================
    #     # Launch a node called "command_arm"
    # =============================================================================
    rospy.init_node('command_arm', anonymous=False)
    
    # Set up subscriber that listens to "joint_angles_desired"
    sub_joint_angles = rospy.Subscriber('/joint_angles_desired', JointState, compute_servo_commands)
    
    sub_bus_servo_commands = rospy.Subscriber('/servo_commands', JointState, move_servos_and_set_joint_state)
    
    rospy.spin()
    

# =============================================================================
#   # Callback function: receives a desired joint angle
def compute_servo_commands(msg_in): 
    # unpack joint angle settings
    alpha0 = msg_in.position[0]
    beta1 = msg_in.position[1]
    beta2 = msg_in.position[2]
    gamma3 = msg_in.position[3]
    beta4 = msg_in.position[4]
    gamma5 = msg_in.position[5]    
    theta_gripper = msg_in.position[6]
    
    # Interpolate angles to servo microseconds to find servo commands 
    cmd_00 = f_interp_rad_to_cmd_01_params(alpha0)
    cmd_01 = f_interp_rad_to_cmd_12_params(beta1)
    cmd_02 = f_interp_rad_to_cmd_23_params(beta2)
    cmd_03 = f_interp_rad_to_cmd_34_params(gamma3)
    cmd_04 = f_interp_rad_to_cmd_45_params(beta4)
    cmd_05 = f_interp_rad_to_cmd_56_params(gamma5)
    cmd_06 = f_interp_rad_to_cmd_gripper_params(theta_gripper)

    cmd_all = [cmd_00,cmd_01,cmd_02,cmd_03,cmd_04,cmd_05,cmd_06]
    
    # Publish the servo commands
    global servo_commands_msg
    servo_commands_msg.name = ['cmd00','cmd01','cmd02','cmd03','cmd04','cmd05','cmd06']
    servo_commands_msg.position = cmd_all
    servo_commands_msg.header.stamp = rospy.Time.now()
    pub_bus_servo_commands.publish(servo_commands_msg)
    
    
# =============================================================================
#   # Callback function: receives servo commands and publishes /joint_states for the RVIZ simulation
def move_servos_and_set_joint_state(msg_in):
    # unpack the commands coming in. 
    cmd_all = msg_in.position
    cmd_00 = cmd_all[0]
    cmd_01 = cmd_all[1]
    cmd_02 = cmd_all[2]
    cmd_03 = cmd_all[3]
    cmd_04 = cmd_all[4]
    cmd_05 = cmd_all[5]
    cmd_06 = cmd_all[6]
                
    # send the servo commands if (and only if) there's an arm attached. 
    if arm_is_present:
        command_bus_servo(0,np.int(cmd_00))
        command_bus_servo(1,np.int(cmd_01))    
        command_bus_servo(2,np.int(cmd_02))    
        command_bus_servo(3,np.int(cmd_03))    
        command_bus_servo(4,np.int(cmd_04))    
        command_bus_servo(5,np.int(cmd_05))    
        command_bus_servo(6,np.int(cmd_06))    

    global joint_state_msg
    try:
        joint_state_msg.position = arm_model.convert_servo_commands_to_joint_state(cmd_all, arm_is_present)
        joint_state_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint']
        joint_state_msg.header.stamp = rospy.Time.now()
        pub_joint_states.publish(joint_state_msg)
    except: 
        rospy.logerr('ERROR: Value out of range. Not Publishing Joint State.')
        
    
    
# Function to command any Hiwonder Bus Servo with a given command
def command_bus_servo(servo_number, cmd):
    servo_ID = servo_IDs_base_to_tip[servo_number]
    arm.setPosition([ [servo_ID, cmd] ])


# Function to shut down all the servos by sending them zero pulse width (same as no command)
def shutdown_arm():
    arm.servoOff()
    print('Arm is Shut Down.')


if __name__ == '__main__':
    try: 
        main()
    except: 
        traceback.print_exc()
    if arm_is_present:
        shutdown_arm()  # to shut it down. 
