<<<<<<< Updated upstream
#!/usr/bin/env python
=======
#!/usr/bin/env python2
>>>>>>> Stashed changes

from StateMachine.sub import *

'''
interact_pole.py
Will move around a pole to complete prequalification
Assumes it will start at the distance from the pole that you want to hold when you move around it

Works by strafing to the side while rotating to keep the pole centered in the front camera.
Since rotating also changes the direction we're strafing, the result should a roughly circular orbit around the pole. I think.
It also tries to keep the same distance from the pole by tracking the initial size of the pole (in diagonal distance) and roughly maintaining that (+- 20 percent).
This is likely to be error-prone because of inconsistencies in how much of the pole we can see.

Variables:
init_size
    Number representing the size of the pole when first detected. We use this to keep distance from the pole roughly equal.

'''

<<<<<<< Updated upstream
# define state interact_pole
class interact_pole(sub):
    #setting to none indicates that we haven't seen it yet
    self.init_size = None

    def __init__(self):
        smach.State.__init__(self, outcomes=['Around_Pole','Lost_Pole'])

    def execute(self, userdata):
        self.init_state()
        self.last_seen = rospy.get_time()

        continue = true

        while (continue)
            box = gbl.get_box_of_class(gbl.boxes, gbl.current_target)
            if (box != None) and box[1] > .3:  # If the box is good
                #update values
                if self.init_size == None:
                    self.init_size = sub.getDistance(box)
                self.last_seen = rospy.get_time()

            msg = self.init_joy_msg()
            msg.axes[self.axes_dict['vertical']] = self.depth_hold()
            
            num_turns = 0
            pole_on_right = False
            pole_on_left = False
            while num_turns < 3:
                while (not pole_on_right):
                    msg.axes[self.axes_dict['leftright']] = -0.1

                while (not pole_on_left):
                    msg.axes[self.axes_dict['rotate']] = 0.05

        #   old centering code
        #      if center[0] < 0.45:
        #     msg.axes[self.axes_dict['rotate']] = 0.1
        #   elif center[0] > 0.55:
        #     msg.axes[self.axes_dict['rotate']] = -0.1

        #   if center[1] < .45:
        #     if self.get_depth() > 1:
        #       msg.axes[self.axes_dict['vertical']] = gbl.depth_const + 0.2
        #     else: 
        #       msg.axes[self.axes_dict['vertical']] = gbl.depth_const
        #   elif center[1] > .55:
        #     msg.axes[self.axes_dict['vertical']] = gbl.depth_const - 0.2
        #     if box:
        #       if self.getDistance(box[2], box[3], box[4], box[5]) > 0.4:
        #         self.is_close = True


        gbl.current_target = self.class_dict['start_gate']

        return 'Around_Pole' # Transitions to SEARCH_FRONT_GATE
=======
# define state track_gate
class interact_pole(sub):
    init_distance = None

    def __init__(self):
        smach.State.__init__(self, outcomes=['Lost_Gate','Entered_Gate'])

    '''
    init_state(self):
        Override sub init_state method to also set init_distance from the pole and initialize last_seen
    '''
    def init_state(self):
        #get the initial distance
        #if we can't see it at first give it two seconds to grab dis
        self.init_distance = gbl.getDistance(gbl.get_box_of_class(gbl.boxes, gbl.current_target))
        while (init_distance = 0 and rospy.get_time() - last_seen < 3)
            self.init_distance = gbl.getDistance(gbl.get_box_of_class(gbl.boxes, gbl.current_target))
        sub.init_state() #call sub init_state to do the boring stuff

    def execute(self, userdata):
      self.init_state()
      self.last_seen = rospy.get_time()

      #control loop
      while(1):
        msg = self.init_joy_msg()
        box = gbl.get_box_of_class(gbl.boxes, gbl.current_target)
        msg.axes[self.axes_dict['vertical']] = gbl.depth_const

        if (box != None) and box[1] > .3:  # If the box is good
            #update values
            if init_distance = None
                init_distance = sub.getDistance(box)
          self.last_seen = rospy.get_time()
          center = self.getCenter(box)
          msg.axes[self.axes_dict['frontback']] = 0.4

          if center[0] < 0.45:
            msg.axes[self.axes_dict['rotate']] = 0.1
          elif center[0] > 0.55:
            msg.axes[self.axes_dict['rotate']] = -0.1

          if center[1] < .45:
            if self.get_depth() > 1:
              msg.axes[self.axes_dict['vertical']] = gbl.depth_const + 0.2
            else: 
              msg.axes[self.axes_dict['vertical']] = gbl.depth_const
          elif center[1] > .55:
            msg.axes[self.axes_dict['vertical']] = gbl.depth_const - 0.2
            if box:
              if self.getDistance(box[2], box[3], box[4], box[5]) > 0.4:
                self.is_close = True


        elif (rospy.get_time() - self.last_seen) > 2: # If the box is not good OR we haven't seen it in a while
          msg.axes[self.axes_dict['frontback']] = 0
          self.joy_pub.publish(msg)

          if self.is_close:
            self.is_close = False
            return "Entered_Gate" # Transitions to INTERACT_GATE
          else:
            rospy.logwarn("Lost tracking the gate for more than 2 seconds")
            return "Entered_Gate" # DEBUG Purposes Only!
            return "Lost_Gate" # Transitions to SEARCH_FRONT_GATE

        self.joy_pub.publish(msg)

        rospy.sleep(gbl.sleep_time)

	def log(self):
		rospy.loginfo('Executing state TRACK_GATE')
>>>>>>> Stashed changes
