import rospy
from sensor_msgs.msg import FluidPressure

current_pressure = 0
thrust_base = -0.425 #roughly steady
thrust_mod = -0.2 #times difference in depth

def depth_callback(msg): #(msg)
    global current_pressure
    current_pressure = msg.fluid_pressure

def get_depth():
    return (current_pressure - surface_pressure) * 1.019744/100000

def hold_depth(depth):
    current_depth = (current_pressure - surface_pressure) * 1.019744/100000
    print('Depth: {}\tPressure: {} Pa'.format(depth, current_pressure))
    depthdiff= depth-current_depth
    thrust_change = depthdiff * thrust_mod
    thrust = thrust_base + thrust_change
    return thrust

surface_pressure = (101325) #1 atm


rospy.init_node('pressure_listener', anonymous=True)

rospy.Subscriber('/mavros/imu/static_pressure', FluidPressure, depth_callback)

rospy.spin()
