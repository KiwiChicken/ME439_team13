# -*- coding: utf-8 -*-

import numpy as np

# =============================================================================
# Paste in test angles here
# These will be passed to the function if this module is called as standalone (at the bottom)
# =============================================================================
test_angles = [0.244978663127, -0.553231565311, 1.98924862896, 0.0, -1.43601706365, 0.0]
test_angles = [0.0, -np.pi/2., np.pi/2., 0.0, 0.0, 0.0]


def armrobfwdkin(angles_radians): 
    print("input joint angles: "+ str(angles_radians))
    y_rotation_sign = 1  # When we see a positive "beta" value, is it positive rotation about the +y (enter 1) or -y (enter -1) axis? 
    
   
    # =============================================================================
    # Unpack angles
    # =============================================================================
    alpha0 = angles_radians[0]
    beta1 = angles_radians[1] * y_rotation_sign
    beta2 = angles_radians[2] * y_rotation_sign
    gamma3 = angles_radians[3]
    beta4 = angles_radians[4] * y_rotation_sign
    gamma5 = angles_radians[5]    
    
    # =============================================================================
    # # Translations in each Homogeneous Transform
    # =============================================================================
    
    r_01 = np.matrix([0., 0., 0.074]).transpose()
    r_12 = np.matrix([0.010, 0., 0.]).transpose()
    r_23 = np.matrix([0.1005, 0., 0.]).transpose()
    r_34 = np.matrix([0.0627, 0., 0.0758]).transpose()
    r_45 = np.matrix([0,0,0]).transpose()
    r_56 = np.matrix([0,0,0]).transpose()
    r_6end = np.matrix([0.133, 0., -0.003]).transpose()
    
    
    # =============================================================================
    # # Transformation Matrices for Serial-Chain Manipulator
    # =============================================================================
    
    # # Transformation from frame 0 to 1 (+Z axis rotation)
    # "Rotation matrix of frame 1 in frame 0's coordinates" (columns are unit vectors of Frame 1 in Frame 0 coordinates)
    R_01 = np.matrix([ [np.cos(alpha0), -np.sin(alpha0), 0.], 
                         [np.sin(alpha0), np.cos(alpha0), 0.],
                         [       0.,           0.,  1.] ])
    # "Homogeneous Transform of Frame 1 in Frame 0's Coordinates"
    T_01 = np.vstack( (np.hstack( (R_01, r_01) ) , [0., 0., 0., 1.]) )
    
    # # Transformation from frame 1 to 2 (+Y axis rotation)
    R_12 = np.matrix([ [ np.cos(beta1), 0., np.sin(beta1)], 
                       [       0. ,     1.,        0.    ],
                       [-np.sin(beta1), 0., np.cos(beta1)] ])
    T_12 = np.vstack( (np.hstack( (R_12, r_12) ) , [0., 0., 0., 1.]) )
        
    # # Transformation from frame 2 to 3 (+Y axis rotation)
    R_23 = np.matrix([ [ np.cos(beta2), 0., np.sin(beta2)], 
                       [       0. ,     1.,        0.    ],
                       [-np.sin(beta2), 0., np.cos(beta2)] ])
    T_23 = np.vstack( (np.hstack( (R_23, r_23) ) , [0., 0., 0., 1.]) )
        
    # # Transformation from frame 3 to 4 (+X axis rotation)
    R_34 = np.matrix([ [ 1. ,        0.     ,        0.      ], 
                       [ 0. , np.cos(gamma3), -np.sin(gamma3)], 
                       [ 0. , np.sin(gamma3),  np.cos(gamma3)] ])
    T_34 = np.vstack( (np.hstack( (R_34, r_34) ) , [0., 0., 0., 1.]) )
            
    # # Transformation from frame 4 to 5 (+Y axis rotation)
    R_45 = np.matrix([ [ np.cos(beta4), 0., np.sin(beta4)], 
                       [       0. ,     1.,        0.    ],
                       [-np.sin(beta4), 0., np.cos(beta4)] ])
    T_45 = np.vstack( (np.hstack( (R_45, r_45) ) , [0., 0., 0., 1.]) )
            
    # # Transformation from frame 5 to 6 (+X axis rotation)
    R_56 = np.matrix([ [ 1. ,        0.     ,        0.      ], 
                       [ 0. , np.cos(gamma5), -np.sin(gamma5)], 
                       [ 0. , np.sin(gamma5),  np.cos(gamma5)] ])
    T_56 = np.vstack( (np.hstack( (R_56, r_56) ) , [0., 0., 0., 1.]) )
    
    
    # =============================================================================
    # Compute Locations of all link endpoints
    # =============================================================================
    # Vector of Zero from the frame origin in question, augmented with a 1 so it can be used with the Homogeneous Transform
    zerovec = np.matrix([0.,0.,0.,1.]).transpose()
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
    
    print("computed joint angles: %f\n%f\n%f\n%f\n%f\n%f\n%f\n" %(pos_0, pos_1, pos_2, pos_3, pos_4, pos_5, pos_6))
    # =============================================================================
    # Return the resulting endpoint
    # =============================================================================
    endpt = np.asfarray(pos_endpoint).transpose()[0]
    print("forward computed endpt: " +str(endpt))
    return endpt


if __name__ == "__main__": 
    endpoint = armrobfwdkin(test_angles)
    print('endpoint xyz')
    print('['+', '.join( map(str, endpoint) ) +']')