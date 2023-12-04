# -*- coding: utf-8 -*-

import numpy as np

# =============================================================================
# Specify Goal Endpoint Here (in Meters)
# These will be passed to the function if this module is called as standalone (at the bottom)
# =============================================================================
test_endpoint = np.array([0.20, 0.05, 0.])


def armrobinvkin(xyz): 
    print("target xyz: "+ str(xyz))
    y_rotation_sign = 1  # When we see a positive "beta" value, is it positive rotation about the +y (enter 1) or -y (enter -1) axis? 
    
    r_01 = np.matrix([0., 0., 0.074]).transpose()
    r_12 = np.matrix([0.010, 0., 0.]).transpose()
    r_23 = np.matrix([0.1005, 0., 0.]).transpose()
    r_34 = np.matrix([0.0627, 0., 0.0758]).transpose()
    r_45 = np.matrix([0,0,0]).transpose()
    r_56 = np.matrix([0,0,0]).transpose()
    r_6end = np.matrix([0.133, 0., -0.003]).transpose()
    
    # =============================================================================
    # Compute Inverse Kinematics
    # =============================================================================
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
# OLD    
#    dR = R - (r_12[0] + r_6end[0])        # subtract off the x of all the links that are not part of the 2-link kinematic solution. NOTE this only works because the X offsets are known to be positioned in the R direction. 
#    dz = xyz[2] - (r_01[2] + r_12[2] + r_6end[2])   # subtract off the Z of all the links that are not part of the 2-link kinematic solution
    # NEW
    dR = R - (r_12[0] + r_6end[2])        # subtract off the x of all the links that are not part of the 2-link kinematic solution. NOTE this only works because the X offsets are known to be positioned in the R direction. 
    dz = xyz[2] - (r_01[2] + r_12[2] + -r_6end[0])   # subtract off the Z of all the links that are not part of the 2-link kinematic solution
    
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
# New: point it DOWN (add pi/2)
    beta4 = -(beta1+beta2) + np.pi/2
    
    # Return the resulting joint angles
    jntangs = np.asfarray([alpha0, beta1, beta2, gamma3, beta4, gamma5])
    print("inverse computed joint angles: "+ str(jntangs))
    return jntangs
    
if __name__ == "__main__":
    joint_angles = armrobinvkin(test_endpoint)
    print('[alpha0, beta1, beta2, gamma3, beta4, gamma5]')
    print('['+', '.join( map(str, joint_angles) ) +']')  # Funky recommended Python way. 
