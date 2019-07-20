#! /usr/bin/env python2
'''Collection of system constants referenced by the system.

Collecting necessary constants in a single location so that
tweaks can be adjusted more easily. 
'''

# Default joystick message values
DEFAULT_MSG_AXES = (-0.01, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0)
DEFAULT_MSG_BUTTONS = (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

'''Enumerations for mapping function to position in axes and buttons tuples.

Usage:
  1) Pushing the front/back control stick forward:
    from constants import AXES
    from StateMachine import Sub

    jmsg = Sub.init_joy_msg()
    jmsg[AXES['frontback']] = 255
    Sub.joy_pub.publish(jmsg)

  2) Pressing a button:
    from constants import BUTTONS
    from StateMachine import Sub

    jmsg = Sub.init_joy_msg()
    jmsg[BUTTONS['x']] = 1
    Sub.joy_pub.publish(jmsg)
    time.sleep(DEBOUNCE_DELAY)
    jmsg[BUTTONS['x']] = 0
    Sub.joy_pub.publish(jmsg)


'''
#TODO: verify and annotate input ranges for these fields.
AXES = {'rotate': 0,
             'vertical': 1,
             'lt': 2,
             'leftright': 3,
             'frontback': 4,
             'rt': 5,
             'dpad_h': 6,
             'dpad_v': 7}

BUTTONS = {'a': 0,
                'b': 1,
                'x': 2,
                'y': 3,
                'lb': 4,
                'rb': 5,
                'back': 6,
                'start': 7,
                'xbox': 8,
                'lstickpress': 9,
                'rstickpress': 10}

#TODO: update for this year's tasks
CLASSES = {'background': 0,
                'start_gate': 1,
                'pole': 2,
                'path_marker': 3}
                # 'claw': 4,
                # 'die1': 5,
                # 'die2': 6,
                # 'die5': 7,
                # 'die6': 8,
                # 'roulette_wheel': 9,
                # 'red_wheel_side': 10,
                # 'black_wheel_side': 11,
                # 'slot_machine': 12,
                # 'slot_handle': 13,
                # 'r_slot_target': 14,
                # 'y_slot_target': 15,
                # 'r_ball_tray': 16,
                # 'g_ball_tray': 17,
                # 'floating_area': 18,
                # 'r_funnel': 19,
                # 'y_funnel': 20,
                # 'g_chip_dispenser': 21,
                # 'g_chip_plate': 22,
                # 'dieX': 23,
                # 'g_funnel': 24,
                # 'pole':25}


# Joystick Messages function map constants
JOY_MAP = {
    # TODO(travis):Actually map these to configured buttons.
    'LAUNCHER_LEFT': 'y',
    'LAUNCHER_RIGHT': 'x',
}

# Launch Tube geometric offsets
'''Offset(Windage) values to apply just before launching torpedo.

Defines offset in MoA (or just degrees, depending upon launcher accuracy and
offset required) to compensate for launcher mounting position in relation
to sub Line-of-Sight. 

Named after the practice in shooting sports where instead
of adjusting the rifle's sights to align point-of-aim to point-of-strike, the
shooter adjusts their point-of-aim to offset the deviation from zero. Not a dig
at Mechanical's ability to get the launcher to shoot straight, or the torpedoes
to fly(?) straight, but a provision for unforseen environmentals
(cross-currents, shipping damage, etc.).

windage_angle - launcher horizontal angle relative to central axis of aiming
                line.
windage_offset - offset left/right relative to target.
elevation_angle - launcher vertical angle relative to central axis of aimimng
                  line.
elevation_offset - offset up/down relative to target.
'''
LAUNCHER_LEFT_OFFSET = {
    'WINDAGE_OFFSET': 0,
    'ELEVATION_OFFSET': 0,
}
LAUNCHER_RIGHT_OFFSET = {
    'WINDAGE_OFFSET': 0,
    'ELEVATION_OFFSET': 0,
}

CAMERA_FORWARD_CENTER = {'X': 1, 'Z': 1}
CAMERA_UNDER_CENTER = {'X': 1, 'Y': 1}

SLEEP_TIME = 0.05 #the amount of time to sleep before looking for another frame
