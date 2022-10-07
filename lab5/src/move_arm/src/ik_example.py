#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper

def main():
    target_pose = [0.795, 0.180, -0.152, 0, 1, 0, 0]
    drop_pose = [0.651, 0.416, -0.149, 0, 1, 0, 0]
    
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')

    # Initialize gripper and calibrate it
    # rospy.init_node('gripper_test')
    right_gripper = robot_gripper.Gripper('right_gripper')
    print('Calibrating...')
    right_gripper.calibrate()
    rospy.sleep(2.0)

    # Logic
    manipulator(pose=target_pose, gripper=right_gripper, grab=True)
    manipulator(pose=drop_pose, gripper=right_gripper, grab=False)


def manipulator(pose: list, gripper, grab: bool):
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = pose[0]
        request.ik_request.pose_stamped.pose.position.y = pose[1]
        request.ik_request.pose_stamped.pose.position.z = pose[2]        
        request.ik_request.pose_stamped.pose.orientation.x = pose[3]
        request.ik_request.pose_stamped.pose.orientation.y = pose[4]
        request.ik_request.pose_stamped.pose.orientation.z = pose[5]
        request.ik_request.pose_stamped.pose.orientation.w = pose[6]
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ: ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])

                if grab:
                    print('Closing...')
                    try:
                        gripper.calibrate()
                    except:
                        import pdb;pdb.set_trace()
                    print('Should be closed.')
                    rospy.sleep(1.0)
                else:
                    print('Opening...')
                    gripper.open()
                    rospy.sleep(1.0)
                break

        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


# Python's syntax for a main() method
if __name__ == '__main__':
    main()
