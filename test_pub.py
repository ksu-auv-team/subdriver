import rospy
import time
from sensor_msgs.msg import Joy


#publish list of axes and duration to   
rospy.init_node('test_publisher')
pub = rospy.Publisher('axes', Joy, queue_size=2)

while not rospy.is_shutdown():
    print("input axes")
    axes = []
    axe_in = input("input an axis value, type -50 to stop: ")
    while axe_in!=-50:
        axes.append(axe_in)
        axe_in = input("input an axis value, type -50 to stop: ")


    duration = input("input duration: ")

    msg = Joy()
    
    pub.publish(msg)
    rospy.sleep(duration)
    
    msg.axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]
    msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    pub.publish(msg)



