#!/usr/bin/env python3
 
import rospy
import traceback 
import numpy as np
# IMPORT the custom message: 
# we import it "from" the ROS package we created it in (here "me439robot") with an extension of .msg ...
# and actually import the message type by name (here "ME439WaypointXY")
from xarmrob_util.msg import ME439WaypointXYZ
from std_msgs.msg import Bool


# =============================================================================
# # Set waypoints to hit along the path 
# =============================================================================
# Get parameters from rosparam
filename = rospy.get_param('/path_svg_file')
print(filename)

#Get Height offset for safety (do practice runs in the air)
vertical_offset = rospy.get_param('/vertical_offset')

# Use an SVG file to specify the path: 
import parse_svg_for_robot_arm_v02 as psvg
waypoints = psvg.convert_svg_to_waypoints(filename, xlength=0.12, ylength=0.12, rmin=0.080, rmax=0.20)
waypoints = np.vstack( (np.array([0.2015, 0.0, 0.05]), waypoints, np.array([0.2015, 0.0, 0.05]), np.array([0.2015, 0.0, 0.2056])))
print(waypoints)
waypoints[:,2] = waypoints[:,2]+vertical_offset

## Or set them manually:
## Waypoints to hit: a "numpy.array" of [x,y,z] coordinates. 
## Example: Square
#waypoints = np.array([[0.5, 0.,0.],[0.5,0.5,0.],[0.,0.5,0.],[0.,0.,0.]]) 

# =============================================================================
# # END of section on waypoint setting
# =============================================================================


##################################################################
# Run the Publisher
##################################################################
# initialize the current "segment" to be the first one (index 0) # (you could skip segments if you wanted to)
waypoint_number = 0  # for waypoint seeking. 
path_complete = Bool()

# Create the publisher for the topic "waypoint_xyz", with message type "ME439WaypointXYZ"
pub_waypoint_xyz = rospy.Publisher('/waypoint_xyz', ME439WaypointXYZ, queue_size=1)

# Create the publisher for the topic "path_complete", with message type "Bool"
pub_path_complete = rospy.Publisher('/path_complete', Bool, queue_size=1)



# Publish desired waypoints at the appropriate time. 
def talker(): 
    global waypoints, waypoint_number, path_complete, pub_waypoint, pub_path_complete
    # Launch a node called "set_waypoints"
    rospy.init_node('set_waypoints', anonymous=False)
 
        # Create a subscriber that listens for messages on the "waypoint_complete" topic
    sub_waypoint_complete = rospy.Subscriber('/waypoint_complete', Bool, increment_waypoint)
    
    # Declare the message to publish. 
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function. 
    # We could also put data in it right away using . 
    msg_waypoint = ME439WaypointXYZ()
   

    
    # set up a rate basis to keep it on schedule.
    r = rospy.Rate(10) # N Hz
    try: 
        # start a loop 
        while not rospy.is_shutdown() and not path_complete.data:
            msg_waypoint.xyz = waypoints[waypoint_number]
            
            # Actually publish the message
            pub_waypoint_xyz.publish(msg_waypoint)
            # Log the info (optional)
#            rospy.loginfo(msg_waypoint)    
            
            pub_path_complete.publish(path_complete)
            
            r.sleep()


    except Exception:
        traceback.print_exc()
        pass
        
    # When it gets here, the loop is broken by the path being complete. Publish that fact. 
    pub_path_complete.publish(path_complete)
        
        


def increment_waypoint(msg_in):
    # get access to the globals set at the top
    global waypoint_number, path_complete, pub_waypoint, pub_path_complete
    
    if msg_in.data :  # it's Boolean
        waypoint_number = waypoint_number + 1
    
    if waypoint_number >= waypoints.shape[0]:
        path_complete.data = True
        waypoint_number = waypoint_number - 1  # This line prevents an array out of bounds error to make sure the node stayed alive. If commented, allow it to increment past the end, which will throw an exception (array out of bounds) the next time it publishes a waypoint and cause the node to die. 
    else:
        path_complete.data = False
    
    pub_path_complete.publish(path_complete)



if __name__ == '__main__':
    try: 
        talker()
    except rospy.ROSInterruptException: 
        pass
