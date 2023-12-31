#!/usr/bin/env python3

# =============================================================================
# based on manual_endpoint_location.py
# takes in an end point and grip/drop and returns a bool once done
# =============================================================================

import numpy as np
import rospy
import traceback
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from endpoint.msg import endpoint # TODO: this is in another pkg

import FwdKinArmRob_serial as FK
import InvKinArmRob_serial as IK

# Load parameters from rosparam to keep handy for the functions below: 
# Matched lists of angles and microsecond commands
map_ang_rad_01 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_01')))
map_ang_rad_12 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_12')))
map_ang_rad_23 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_23')))
map_ang_rad_34 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_34')))
map_ang_rad_45 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_45')))
map_ang_rad_56 = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_joint_56')))
map_ang_rad_gripper = np.radians(np.array(rospy.get_param('/rotational_angles_for_mapping_gripper')))

# limits for each of the joints
# keren: update here to enlarge workspace
rotlim_01 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_01')))
rotlim_12 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_12')))
rotlim_23 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_23')))
rotlim_34 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_34')))
rotlim_45 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_45')))
rotlim_56 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_56')))
rotlim_gripper = np.radians(np.array(rospy.get_param('/gripper_limits')))


# Create the message
cmds = [0., -np.pi/2., np.pi/2., 0., 0., 0., np.pi/2.]
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint', 'gripper'];
joint_angles_desired_msg.position = cmds      # upright neutral position

arm_complete = Bool()
endpoint_xyz = np.array()
is_grip = Bool()

def manual_endpoint_location(): 
    global sub_arm_xyz, pub_arm_complete, arm_complete
    global endpoint_xyz, is_grip
    rospy.init_node('manual_endpoint_locations',anonymous=False)

    # Create a subscriber that listens for messages on the "arm_xyz" topic
    sub_arm_xyz = rospy.Subscriber('/arm_xyz', endpoint, operate_arm)
    # Create the publisher for the topic "arm_xyz", with message type "endpoint"
    pub_arm_complete = rospy.Publisher('/arm_complete', Bool, queue_size=1)
    # Create the publisher. Name the topic "joint_angles_desired", with message type "JointState"
    pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)

    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(10) # N Hz
    go_anyway = 'N'
    try: 
        while not rospy.is_shutdown():
            xyz_goal = endpoint_xyz
            # Compute Inverse Kinematics
            ang = IK.armrobinvkin(xyz_goal)
            
            # Compute limited joint angles. 
            ang_lim = ang
            ang_lim[0] = np.clip(ang[0], np.min(rotlim_01), np.max(rotlim_01))
            ang_lim[1] = np.clip(ang[1], np.min(rotlim_12), np.max(rotlim_12))
            ang_lim[2] = np.clip(ang[2], np.min(rotlim_23), np.max(rotlim_23))
            ang_lim[3] = np.clip(ang[3], np.min(rotlim_34), np.max(rotlim_34))
            ang_lim[4] = np.clip(ang[4], np.min(rotlim_45), np.max(rotlim_45))
            ang_lim[5] = np.clip(ang[5], np.min(rotlim_56), np.max(rotlim_56))
            
            # Predict where the "limited" angles will get you. 
            xyz_pred = FK.armrobfwdkin(ang_lim)
            
            xyz_err_pred = xyz_goal-xyz_pred  
            xyz_err_norm = np.sqrt(  np.sum(  np.power(xyz_err_pred, 2) ) )
            if np.isnan(xyz_err_norm) or (xyz_err_norm > 0.001):
                rospy.loginfo('Unreachable Endpoint!')
                if not (go_anyway[0].upper() == 'Y') : 
                    rospy.loginfo('Not moving - continue.')

            # If the program gets here it has been told to go ahead. 
            # Move to endpoint. 
            ang_lim = np.append(ang_lim, 0.) ## append a Gripper angle. This needs updating. 
            joint_angles_desired_msg.position = ang_lim 
            joint_angles_desired_msg.header.stamp = rospy.Time.now()
            pub_joint_angles_desired.publish(joint_angles_desired_msg)
            # TODO: also grip or drop
            rospy.loginfo('Moving to {}'.format(ang_lim))
            rospy.loginfo('Predicted location: \n{}'.format(xyz_pred))
            #TODO: lazy way to wait completion
            r.sleep(5)
            pub_arm_complete.publish(True)

            r.sleep()
    except Exception:
        traceback.print_exc()
        pass
        
            
            
def operate_arm(msg_in):
    global endpoint, is_grip
    endpoint_xyz = np.array([msg_in.x,msg_in.y,msg_in.z])
    is_grip = msg_in.grip()
    print('Target {}'.format(endpoint_xyz))
    return

if __name__ == "__main__":
    try:
        manual_endpoint_location()
    except:
        traceback.print_exc()
        pass
    
    
