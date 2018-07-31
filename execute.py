import rospy
import numpy as np

# box_subscriber 

# create dict of statuses and whether they've been completed
# that is, track what we've done 
completed = dict.fromkeys(['start_gate_found', 'start_gate_passed', 'dice_found', 'dice_hit'], False)
print(completed)

def img_callback():


def main():
    
    
    #add killswitch check here

    rospy.spin()



if __name__ == '__main__'
    main()