import sys
import rospy
from submarine_msgs_srvs.msg import Detections
from submarine_msgs_srvs.msg import Sub_status
sys.path.append('./StateMachine/')
import gbl


def talker():
    status = Sub_status()
    gbl.depth = 0
    rospy.init_node('sub_gbl')
    status_pub = rospy.Publisher('sub_gbl', Sub_status, queue_size = 1)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        run_start_time = None
        status.depth = gbl.depth #current depth in meters
        status.init_depth = gbl.init_depth #depth at beginning of run - should be near 0
        status.heading = gbl.heading #current compass heading in degrees from 0-360
        status.init_heading = gbl.init_heading #compass heading at beginning of run
        status.state_heading = gbl.state_heading
        status.detections_front = gbl.detections_front #list of detections from the front camera that will be filled by the neural network
        status.num_detections_front = gbl.num_detections_front #why would we not just use len(detections)?
        status.detections_bottom = gbl.detections_bottom #list of detections from the bottom camera that will be filled by the neural network
        status.num_detections_bottom = gbl.num_detections_bottom #why would we not just use len(detections)?
        status.current_target = gbl.current_target #the current object being targeted TODO: allow multiple targets
        status.surfacing = gbl.surfacing
        status.debug = gbl.debug
        
	gbl.depth = gbl.depth + 0.1
        status_pub.publish(status)
        rate.sleep()
        
        
if __name__ == '__main__':
    talker()
