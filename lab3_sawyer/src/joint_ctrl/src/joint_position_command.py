#! /usr/bin/env python
# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
SDK Joint Position Example: keyboard
"""
import argparse
from time import sleep

import rospy
import numpy as np

import intera_interface
import intera_external_devices

from intera_interface import CHECK_VERSION

def get_j(limb):
    joint_names = limb.joint_names()
    pos = [limb.joint_angle(n) for n in joint_names]
    return pos

def map_keyboard(side):
    limb = intera_interface.Limb(side)

    try:
        gripper = intera_interface.Gripper(side + '_gripper')
    except:
        has_gripper = False
        rospy.loginfo("The electric gripper is not detected on the robot.")
    else:
        has_gripper = True

    joints = limb.joint_names()

    def set_j(limb, joint_name, cmd_position):
        joint_command = {joint_name: cmd_position}
        print("Executing" + str(joint_command))
        limb.set_joint_position_speed(0.3)
        limb.set_joint_positions(joint_command)

    def set_j_dict(limb, joint_command):
        print("Executing" + str(joint_command))
        limb.set_joint_position_speed(0.3)
        limb.set_joint_positions(joint_command)

    def set_g(action):
        if has_gripper:
            if action == "close":
                gripper.close()
            elif action == "open":
                gripper.open()
            elif action == "calibrate":
                gripper.calibrate()

    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")

    while not rospy.is_shutdown():
        command_list = input("Command joint angles in a list. The list has to be [1,2,3,4] format for 7 angles: ")
        command_list = [float(c) for c in command_list[1:-1].split(',')]

        command_map = {joints[i]: command_list[i] for i in range(len(command_list))}
                
        pos = get_j(limb)        
        while (np.linalg.norm(np.array(pos) - np.array(command_list)) > 0.05) and not rospy.is_shutdown():
            set_j_dict(limb, command_map)
            pos = get_j(limb)
            rospy.sleep(0.1)



def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
    See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the joint position keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()
    print('Limb to control: ', args.limb)
    map_keyboard(args.limb)
    print("Done.")


if __name__ == '__main__':
    main()
