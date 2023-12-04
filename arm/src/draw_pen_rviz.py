#! /usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import JointState
import tf_conversions
import tf2_ros
import math
import traceback
import time
rospy.init_node('draw_pen_rviz')


pen_z = -.035
z_lim = .02 #how far you can go up and down and still get markers on the path.
pathInterval = .005 #how many meters before making another path point (2cm now).
path_distance = 0.0
max_path_pts = 100 #adjust this if you bog down on raspberry pi.
markerArray = MarkerArray()
last_x = 0.0
last_y = 0.0
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer);



def makeMarker(xcoord,ycoord):
    marker = Marker()
    marker.header.frame_id="base_link"
    marker.type=marker.SPHERE
    marker.action=marker.ADD
    marker.scale.x = .005
    marker.scale.y = .005
    marker.scale.z = .005
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.b = 0.0
    marker.pose.position.x = xcoord
    marker.pose.position.y = ycoord
    marker.pose.position.z = 0.0
    return marker


def updatePenMarks(msg):
    global path_distance
    global last_point
    global markerPublisher
    global last_x
    global last_y
    x = msg.transform.translation.x
    y = msg.transform.translation.y
    z = msg.transform.translation.z+pen_z
    print("X: " + str(x) + " Y: " + str(y) + " Z: " + str(z))
    if (z < z_lim and z > -z_lim):
        path_distance  = path_distance + math.sqrt((x-last_x)*(x-last_x) + (y-last_y)*(y-last_y))
        last_x = x
        last_y = y
        if (path_distance > pathInterval):
            markerArray.markers.append(makeMarker(x,y))
            path_distance = 0
            if (len(markerArray.markers)>max_path_pts):
                markerArray.markers.pop(0)
            id = 0
            for m in markerArray.markers:
                m.id = id
                id = id +1
            markerPublisher.publish(markerArray)
        

def main():
    global markerPublisher
    markerPublisher = rospy.Publisher('/marker_array',MarkerArray, queue_size=1)
    
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('base_link','fingers',rospy.Time.now(),rospy.Duration(1.0))
            print("got transform!")
            updatePenMarks(trans);
        except Exception as e:
            print(e)
            continue
    time.sleep(.1)

if __name__ == '__main__':
    try: 
        main()
    except rospy.ROSInterruptException: 
        traceback.print_exc()
        pass
    
    
