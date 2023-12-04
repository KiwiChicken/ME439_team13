#!/usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom messages: 
from xarmrob_util.msg import ME439WaypointXYZ 
from sensor_msgs.msg import JointState

# global variable to hold the waypoint currently being tracked
waypoint = ME439WaypointXYZ()
waypoint.xyz = np.array([0.,0.,0.])


# Load parameters from rosparam to keep handy for the functions below: 
# Sign of 'positive' rotations w.r.t. the y axis
y_rotation_sign = np.sign(rospy.get_param('/y_rotation_sign'))
# Vectors from each frame origin to the next frame origin, in the proximal
r_01 = np.matrix(rospy.get_param('/frame_offset_01')).transpose()
r_12 = np.matrix(rospy.get_param('/frame_offset_12')).transpose()
r_23 = np.matrix(rospy.get_param('/frame_offset_23')).transpose()
r_34 = np.matrix(rospy.get_param('/frame_offset_34')).transpose()        
r_45 = np.matrix(rospy.get_param('/frame_offset_45')).transpose()
r_56 = np.matrix(rospy.get_param('/frame_offset_56')).transpose()
r_6end = np.matrix(rospy.get_param('/endpoint_offset_in_frame_6')).transpose()

# limits for each of the joints
rotlim_01 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_01')))
rotlim_12 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_12')))
rotlim_23 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_23')))
rotlim_34 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_34')))
rotlim_45 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_45')))
rotlim_56 = np.radians(np.array(rospy.get_param('/rotational_limits_joint_56')))

# Create the publisher. Name the topic "joint_angles_desired", with message type "JointState"
pub_joint_angles_desired = rospy.Publisher('/joint_angles_desired', JointState, queue_size=1)

# Create the message
joint_angles_desired_msg = JointState()
joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint','gripper'];
joint_angles_desired_msg.position = [0., -np.pi/2., np.pi/2., 0., -np.pi/2., 0., np.pi/2.]      # upright neutral position



def main(): 
    # Actually launch a node called "inverse_kinematics"
    rospy.init_node('inverse_kinematics', anonymous=False)
    
    # Subscriber to the "target_xyz" topic
    sub_waypoint = rospy.Subscriber('/target_xyz', ME439WaypointXYZ, compute_joint_angles)
      
    # Spin() to keep the node from exiting
    rospy.spin()
    


# =============================================================================
# # Function to update the path to the waypoint based on the robot's current position
# =============================================================================
def compute_joint_angles(endpoint_msg_in):  
#    print(endpoint_msg_in.xyz)
    # First compute the inverse kinematics for perfect endpoint positioning
    joint_angles_IK = compute_IK(endpoint_msg_in)
#    print(joint_angles_IK)
    
    # Then Limit the angle at each joint to its achievable range
    angles_limited = limit_joint_angles(joint_angles_IK)
    angles_limited = np.append(angles_limited,0)
    
    # Pack the joint angles and publish the message
    joint_angles_desired_msg.position = angles_limited
    joint_angles_desired_msg.header.stamp = rospy.Time.now()
    pub_joint_angles_desired.publish(joint_angles_desired_msg)
    
#    rospy.loginfo(joint_angles_desired_msg)


    
def compute_IK(endpoint_msg_in):
    xyz = endpoint_msg_in.xyz
    
    # Compute base rotation plus 2-link IK... 
    # Assuming that the "forearm" and "fingers" do not rotate 
    gamma3 = 0
    gamma5 = 0
    # ... and assume the wrist is controlled so the "fingers" point directly in the radial direction
    # (in order to hold the "marker" directly vertical)
    # THIS WOULD ALL CHANGE if the wrist is controlled in some other way. 

    # ... and assume the wrist is controlled so the "fingers" point directly in the radial direction
    # (in order to hold the "marker" directly vertical)
    # THIS WOULD ALL CHANGE if the wrist is controlled in some other way. 
    
    # First the out-of-plane rotation
    alpha0 = np.arctan2(xyz[1], xyz[0])
    
    # Now compute the radial and vertical distances spanned by the two links of the arm
    R = np.linalg.norm(xyz[0:2])   # Remember that this means "start at 0, stop BEFORE 2"
    dR = R - (r_12[0] + r_6end[2])        # subtract off the x of all the links that are not part of the 2-link kinematic solution. NEW 2023: Hand is held "down" (+pi/2 rad) so Z offset is known to be positioned in the R direction. 
    
    dz = xyz[2] - (r_01[2] + r_12[2] + -r_6end[0])   # subtract off the Z of all the links that are not part of the 2-link kinematic solution. NEW 2023: Hand is held "down" (+pi/2 rad) so X offset is known to be positioned in the -Z direction.
    
    # Now compute the "overall elevation" angle from the "shoulder" to the "wrist" 
    # NOTE this assumes rotations about the +y axis (positive rotations push the wrist down)
    psi = -np.arctan2(dz, dR)  # use negative because of the positive-rotations-down convention. 
    # Now the difference between the actual shoulder angle and the overall elevation angle
    # ... being aware that there are two solutions and we want the "elbow up" configuration. 
    L1 = np.linalg.norm(r_23)  # vector magnitude of the link that spans from shoulder to elbow ("upper arm")
    L2 = np.linalg.norm(r_34)  # vector magnitude of the link that spans from elbow to wrist ("lower arm")
    H = np.linalg.norm(np.array((dz,dR))) # vector magnitude of the vector from shoulder to wrist. (H = hypotenuse)
    phi = np.arccos( (L2**2 - L1**2 - H**2)/(-2*L1*H) )  # arccos will always return a positive value. 
    
    # Compute the "elbow up" solution for beta1
    beta1 = psi - phi   #  phi is always positive (from arccos function) so "-phi" is the elbow pose in a more negative position (elbow up for the +y axis rotations) 
    
    # Compute the corresponding solution for beta2VL (VL = "virtual link" direct from joint 2 to joint 3 (elbow to wrist)
    # Use the ArcTangent (two quadrant)
    beta2VL = np.arctan2(H*np.sin(phi), H*np.cos(phi)-L1)
#    print(beta2VL)
    
    # Compute the offset in angle between  the VL (virtual link straight from joint 3 to joint 4) and the true link axis. 
    # True link should be more positive by this amount. 
    beta2_offset_from_VL = np.arctan2(r_34[2], r_34[0])  
    
    # Real-world beta2, assuming +y axis rotations
    beta2 = beta2VL + beta2_offset_from_VL 
       
    # Depending on the sign of positive rotations, give back the rotations. 
    beta1 = beta1 * y_rotation_sign
    beta2 = beta2 * y_rotation_sign
    
    # Compute beta4 to cancel out beta1 and beta2 (works regardless of the sign) 
    beta4 = -(beta1+beta2) + np.pi/2

    
    return np.array([alpha0, beta1, beta2, gamma3, beta4, gamma5])

    
def limit_joint_angles(angles):
    angles_limited = angles
    
    # Clip (saturate) the angles at the achievable limits. 
    angles_limited[0] = np.clip(angles_limited[0], np.min(rotlim_01), np.max(rotlim_01))
    angles_limited[1] = np.clip(angles_limited[1], np.min(rotlim_12), np.max(rotlim_12))
# special case for angle 2 below, skip it here:
# Reinstated 2019 with modified arms
    angles_limited[2] = np.clip(angles_limited[2], np.min(rotlim_23), np.max(rotlim_23))
    angles_limited[3] = np.clip(angles_limited[3], np.min(rotlim_34), np.max(rotlim_34))
    angles_limited[4] = np.clip(angles_limited[4], np.min(rotlim_45), np.max(rotlim_45))
    angles_limited[5] = np.clip(angles_limited[5], np.min(rotlim_56), np.max(rotlim_56))

## Removed 2019    -  No longer a palletizing robot. 
#    # Also a version of beta2 called beta2WORLD that accounts for the fact that Link 2 is 
#    # controlled by a parallelogram structure. This means beta2WORLD = beta2 + beta1. 
#    # beta2WORLD is the one that should be sent to the Joint. 
#    beta1 = angles_limited[1]
#    beta2 = angles_limited[2]
#    beta2WORLD = beta1 + beta2
#    # beta2WORLD has special limits that depend on beta1
#    # nuanced negative limit for beta2World are a mess. Use a reasonable limit instead: 
#    beta2WORLD_min = np.min( rotlim_23 )
#    # Nuanced max limit is not as bad. 
#    beta2WORLD_max = np.min( [np.max(rotlim_23) + (beta1-np.max(rotlim_12)), np.max(rotlim_23)] ) 
#    beta2WORLD_limited = np.clip(beta2WORLD, beta2WORLD_min, beta2WORLD_max)    
#    
#    beta2_limited = beta2WORLD - beta1
#    angles_limited[2] = beta2_limited
#    
##    # Alternative: return a version that has "beta2WORLD" (limited version) in the [2] element
##    angles_limited[2] = beta2WORLD_limited
    
    return angles_limited




if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
