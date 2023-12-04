#!/usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom messages: 
from xarmrob_util.msg import ME439WaypointXYZ 
from std_msgs.msg import Bool


# global variable to hold the waypoint currently being tracked
waypoint = ME439WaypointXYZ()
waypoint.xyz = (0.2015, 0.0, 0.2056)     # Set to 0 initially so it does not think the job is finished. 

# global variable to hold the previous position the robot was in
# assumes it was just the previous commanded target point. 
position_current = ME439WaypointXYZ()
position_current.xyz = (0.2015, 0.0, 0.2057)    # Set to 0 initially so it does not think the job is finished. 

isWaypointReady = False;

# Global to track the state of completion of the  waypoint    
waypoint_complete = Bool()
waypoint_complete.data = False    

# =============================================================================
#     Set up a waypoint seeker
#     and to "waypoint_xyz"  (ME439WaypointXYZ)
#     Publish "target_xyz" (ME439WaypointXYZ) (the "smoothed" instantaneous goal)
# =============================================================================

# Get parameters from rosparam
endpoint_speed = rospy.get_param('/endpoint_speed') # how fast should we move?  
command_frequency = rospy.get_param('/command_frequency') # how many times per second should we send a new command to the Arm? 
drmax = endpoint_speed / command_frequency

# Create the publisher. Name the topic "target_xyz", with message type "ME439WaypointXYZ"
pub_target_xyz = rospy.Publisher('/target_xyz', ME439WaypointXYZ, queue_size=1)

# Create the publisher for the topic "waypoint_complete", with message type "Bool"
pub_waypoint_complete = rospy.Publisher('/waypoint_complete', Bool, queue_size=1)


# Publish desired targets at the appropriate time. 
def talker(): 
    # Actually launch a node called "waypoint_seeker"
    rospy.init_node('waypoint_seeker', anonymous=False)
    
    # Subscriber to the "waypoint_xyz" topic
    sub_waypoint = rospy.Subscriber('/waypoint_xyz', ME439WaypointXYZ, set_waypoint)
      
    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(command_frequency)
    
    # Send the arm to an initial target prior to receiving its first waypoint: 
    initial_target = ME439WaypointXYZ() 
    initial_target.xyz = position_current.xyz
    pub_target_xyz.publish(initial_target)
    
    # start the periodic calls to the target publisher. 
    while not rospy.is_shutdown():
        set_next_target_toward_waypoint()
        
        r.sleep()
    


# =============================================================================
# # Function to update the path to the waypoint based on the robot's current position
# =============================================================================
def set_next_target_toward_waypoint():
    # First assign the incoming message
    global position_current, waypoint, waypoint_complete, isWaypointReady
    if(isWaypointReady):
        # set the path to be directly from here to the waypoint
        dr = np.array( waypoint.xyz ) - np.array( position_current.xyz  )   # XYZ vector from current location to Waypoint
        
        drlength = np.sqrt(dr.dot(dr))  # Compute the Cartesian distance to the Waypoint. 
        drhat = dr/drlength
      
        # Compute the ME439WaypointXYZ message        
        target = ME439WaypointXYZ()        
        if (drlength < drmax) :     # If close enough to reach the waypoint, go there and set it as complete
            target = waypoint
            if not waypoint_complete.data: # Only publish the "waypoint_complete" topic when first detected. 
                waypoint_complete.data = True
                pub_waypoint_complete.publish(waypoint_complete)
        # Otherwise, go one step toward the waypoint
        else:   
            drtarget = drmax*drhat
            target.xyz = position_current.xyz + drtarget
            
        # Update "previous location"
        position_current = target
        
        #  Publish it
        pub_target_xyz.publish(target)
        
    # Else waypoint is not ready (hasn't started up)
    else:
        pass  # Do Nothing



# Function to receive a Waypoint and set the goal point to it.     
def set_waypoint(waypoint_msg_in): 
    global waypoint, waypoint_complete, isWaypointReady
    waypoint = waypoint_msg_in
    waypoint_complete.data = False
    isWaypointReady = True


        
    

if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
