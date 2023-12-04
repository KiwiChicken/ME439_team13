#!/usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom messages: 
from xarmrob_util.msg import ME439JointsXYZ 
from sensor_msgs.msg import JointState

# Load parameters from rosparam to keep handy for the functions below: 
# Sign of 'positive' rotations w.r.t. the y axis
y_rotation_sign = np.sign(rospy.get_param('/y_rotation_sign'))
# Vector from frame 0 origin to frame 1 origin in Frame 0
r_01 = np.matrix(rospy.get_param('/frame_offset_01')).transpose()
r_12 = np.matrix(rospy.get_param('/frame_offset_12')).transpose()
r_23 = np.matrix(rospy.get_param('/frame_offset_23')).transpose()
r_34 = np.matrix(rospy.get_param('/frame_offset_34')).transpose()        
r_45 = np.matrix(rospy.get_param('/frame_offset_45')).transpose()
r_56 = np.matrix(rospy.get_param('/frame_offset_56')).transpose()
r_6end = np.matrix(rospy.get_param('/endpoint_offset_in_frame_6')).transpose()

# Vector of Zero from the frame origin in question, augmented with a 1 so it can be used with the Homogeneous Transform
zerovec = np.matrix([0.,0.,0.,1.]).transpose()
    
# Create the publishers. Name the topic "endpoint_xyz", with message type "ME439JointsXYZ"
pub_endpoint_xyz = rospy.Publisher('/endpoint_xyz', ME439JointsXYZ, queue_size=1)
pub_joints_xyz = rospy.Publisher('/joints_xyz', ME439JointsXYZ, queue_size=1)


def main(): 
    # Actually launch a node called "forward_kinematics"
    rospy.init_node('forward_kinematics', anonymous=False)
    
    sub_joint_states = rospy.Subscriber('/joint_states', JointState, calculate_joint_positions)
    
    rospy.spin()
    



def calculate_joint_positions(msg_in): 
    T_01, T_12, T_23, T_34, T_45, T_56 = build_transforms(msg_in)
    
    pos_0 = zerovec[0:3,0] # base link location: 0
    pos_1 = (T_01*zerovec)[0:3,0]
    T_02 = T_01*T_12
    pos_2 = (T_02*zerovec)[0:3,0]
    T_03 = T_02*T_23
    pos_3 = (T_03*zerovec)[0:3,0]
    T_04 = T_03*T_34
    pos_4 = (T_04*zerovec)[0:3,0]
    T_05 = T_04*T_45
    pos_5 = (T_05*zerovec)[0:3,0]
    T_06 = T_05*T_56
    pos_6 = (T_06*zerovec)[0:3,0]
    pos_endpoint = (T_06*np.vstack((r_6end,1)) )[0:3,0]
    
    
    pos_all = np.hstack( (pos_0, pos_1, pos_2, pos_3, pos_4, pos_5, pos_6, pos_endpoint) )
    
    
    # Publish the computed endpoint location...
    endpoint_msg = ME439JointsXYZ()
    endpoint_msg.x = pos_endpoint[0]
    endpoint_msg.y = pos_endpoint[1]
    endpoint_msg.z = pos_endpoint[2]
    pub_endpoint_xyz.publish(endpoint_msg)
    
    # And all the joint locations
    joints_msg = ME439JointsXYZ()
    joints_msg.x = np.array(pos_all)[0]
    joints_msg.y = np.array(pos_all)[1]
    joints_msg.z = np.array(pos_all)[2]
    pub_joints_xyz.publish(joints_msg) 
    

    
def build_transforms(msg_in):
    angles_radians =msg_in.position
    # Unpack joint angles.
    alpha0 = angles_radians[0]
    beta1 = angles_radians[1] * y_rotation_sign
    beta2 = angles_radians[2] * y_rotation_sign
    gamma3 = angles_radians[3]
    beta4 = angles_radians[4] * y_rotation_sign
    gamma5 = angles_radians[5]    
    
    # =============================================================================
    # # Transformation from frame 0 to 1 (+Z axis rotation)
    # =============================================================================
    # "Rotation matrix of frame 1 in frame 0's coordinates" (columns are unit vectors of Frame 1 in Frame 0 coordinates)
    R_01 = np.matrix([ [np.cos(alpha0), -np.sin(alpha0), 0.], 
                         [np.sin(alpha0), np.cos(alpha0), 0.],
                         [       0.,           0.,  1.] ])
    # "Homogeneous Transform of Frame 1 in Frame 0's Coordinates"
    T_01 = np.vstack( (np.hstack( (R_01, r_01) ) , [0., 0., 0., 1.]) )
    
    # =============================================================================
    # # Transformation from frame 1 to 2 (+Y axis rotation)
    # =============================================================================
    R_12 = np.matrix([ [ np.cos(beta1), 0., np.sin(beta1)], 
                       [       0. ,     1.,        0.    ],
                       [-np.sin(beta1), 0., np.cos(beta1)] ])
    T_12 = np.vstack( (np.hstack( (R_12, r_12) ) , [0., 0., 0., 1.]) )
        
    # =============================================================================
    # # Transformation from frame 2 to 3 (+Y axis rotation)
    # =============================================================================
    R_23 = np.matrix([ [ np.cos(beta2), 0., np.sin(beta2)], 
                       [       0. ,     1.,        0.    ],
                       [-np.sin(beta2), 0., np.cos(beta2)] ])
    T_23 = np.vstack( (np.hstack( (R_23, r_23) ) , [0., 0., 0., 1.]) )
        
    # =============================================================================
    # # Transformation from frame 3 to 4 (+X axis rotation)
    # =============================================================================
    R_34 = np.matrix([ [ 1. ,        0.     ,        0.      ], 
                       [ 0. , np.cos(gamma3), -np.sin(gamma3)], 
                       [ 0. , np.sin(gamma3),  np.cos(gamma3)] ])
    T_34 = np.vstack( (np.hstack( (R_34, r_34) ) , [0., 0., 0., 1.]) )
            
    # =============================================================================
    # # Transformation from frame 4 to 5 (+Y axis rotation)
    # =============================================================================
    R_45 = np.matrix([ [ np.cos(beta4), 0., np.sin(beta4)], 
                       [       0. ,     1.,        0.    ],
                       [-np.sin(beta4), 0., np.cos(beta4)] ])
    T_45 = np.vstack( (np.hstack( (R_45, r_45) ) , [0., 0., 0., 1.]) )
            
    # =============================================================================
    # # Transformation from frame 5 to 6 (+X axis rotation)
    # =============================================================================
    R_56 = np.matrix([ [ 1. ,        0.     ,        0.      ], 
                       [ 0. , np.cos(gamma5), -np.sin(gamma5)], 
                       [ 0. , np.sin(gamma5),  np.cos(gamma5)] ])
    T_56 = np.vstack( (np.hstack( (R_56, r_56) ) , [0., 0., 0., 1.]) )
    
    return T_01, T_12, T_23, T_34, T_45, T_56 





if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
