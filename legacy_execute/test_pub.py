import rospy
import time
from sensor_msgs.msg import Joy


#publish list of axes and duration to   
rospy.init_node('test_publisher')
pub = rospy.Publisher('joy', Joy, queue_size=2)

while not rospy.is_shutdown():
    print("input axes")
    axes = []
    axe_in = input("input a down value, type -50 to stop: ")


    duration = input("input duration: ")

    msg = Joy()
    msg.axes = [0.0, axe_in, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    pub.publish(msg)
    rospy.sleep(duration)
    
    msg.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub.publish(msg)



