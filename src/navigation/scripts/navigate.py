#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
import tf

OUTCOME_SUCCESS = "success"
OUTCOME_FAILURE = "failed"


def main():
    rospy.init_node("navigation_smach")


    #navigation state machine
    sm = smach.StateMachine(outcomes=[OUTCOME_SUCCESS, OUTCOME_FAILURE, "aborted", "preempted"])
    sm.userdata.goal = None
    sm.userdata.location = None
    sm.userdata.laser_scan_count = 0
    sm.userdata.laser_scans = []

    def gen_mb_goal(pose: Pose) -> MoveBaseGoal:
        """Helper function to generate a MoveBaseGoal message from a Pose"""
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.pose = pose
        nav_goal.target_pose.header.frame_id = "map"

        return nav_goal

    def turn_180(pose: Pose) -> PoseStamped:
        """Helper function to turn 180 degrees"""
        pose.orientation.x *= -1
        pose.orientation.y *= -1
        pose.orientation.z *= -1
        pose.orientation.w *= -1
        return gen_mb_goal(pose)

    with sm:
        q_180_values = tf.transformations.quaternion_from_euler(3.14159, 0, 0);
        q_180_values = q_180_values.tolist()
        q_180 = Quaternion(x=q_180_values[0], y=q_180_values[1], z=q_180_values[2], w=q_180_values[3])


        # hardcoded Poses for the 2 double-post areas and the Home base, obtained manualy from RVIZ
        pose_a = Pose(
            position = Point(
            x=8.110886573791504,
            y=-8.246213912963867,
            z=0.0),
            orientation = Quaternion(x=0, y=0, z=0, w=1)
        )

        pose_a_180 = Pose(
            position = Point(
            x=8.110886573791504,
            y=-8.246213912963867,
            z=0.0),
            orientation = q_180 
        )

        pose_b = Pose(
            position = Point(
            x=5.485304832458496,
            y=5.356661319732666,
            z=0.0),
            orientation = Quaternion(x=0, y=0, z=0, w=1)
        )

        pose_b_180 = Pose(
            position = Point(
            x=5.485304832458496,
            y=5.356661319732666,
            z=0.0),
            orientation = q_180 
        )

        pose_home = Pose(
            position = Point(
            x=1.1931631565093994,
            y=-3.3397326469421387,
            z=0.0),
            orientation = Quaternion(x=0, y=0, z=0, w=1)
        )

        def goal_cb(userdata, goal):
            """This function is called each time the navigation state is entered.
            It uses userdata to determine where the jackal should go dynamically"""

            # frst time going somewhere? go to A
            if userdata.goal is None:
                userdata.goal = "A"
                target_pose = gen_mb_goal(pose_a)
                rospy.loginfo(f"NAVIGATING TO {target_pose}")
                return target_pose

            # go to A
            if userdata.goal == "A":
                target_pose = gen_mb_goal(pose_a)
                rospy.loginfo(f"NAVIGATING TO {target_pose}")
                return target_pose

            # rotate at A
            if userdata.goal == "A_180":
                target_pose = turn_180(pose_a)
                rospy.loginfo(f"NAVIGATING TO {target_pose}")
                return target_pose

            # go to B
            if userdata.goal == "B":
                target_pose = gen_mb_goal(pose_b)
                rospy.loginfo(f"NAVIGATING TO {target_pose}")
                return target_pose

            # rotate at B
            if userdata.goal == "B_180":
                target_pose = turn_180(pose_b)
                rospy.loginfo(f"NAVIGATING TO {target_pose}")
                return target_pose

            # go home
            if userdata.goal == "HOME":
                rospy.loginfo(f"NAVIGATING TO HOME")
                return gen_mb_goal(pose_home)

        def navigation_result_callback(userdata, status, result):
            """ This function is called after a navigation task succeeds or fails, saves the current pose to userdata and sets the transition accordingly"""

            old_goal = userdata.goal

            if status == GoalStatus.SUCCEEDED:

                # got to A? next take a laser scan
                if old_goal == "A":
                    userdata.location = pose_a
                    return "laser_scan"

                # got to A_180? next take another laser scan
                if old_goal == "A_180":
                    userdata.location = pose_a_180
                    return "laser_scan"

                # got to B? next take a laser scan
                if old_goal == "B":
                    userdata.location = pose_b
                    return "laser_scan"

                # got to B_180? next take second laser_scan
                if old_goal == "B_180":
                    userdata.location = pose_b_180
                    return "laser_scan"

                # got to HOME? next proceed to DONE State for printing
                if old_goal == "HOME":
                    userdata.location = pose_home
                    return "succeeded"

            # if it failed and was not already going home
            if old_goal != "HOME":
                userdata.goal = "HOME"
                rospy.loginfo("FAILED TO NAVIGATE, GOING HOME")
                return "failed_go_home"

            # all other scenarios of failure should lead to failure final state
            else:
                return "aborted"

        # single state machine to handle all navigation
        nav_state = smach_ros.SimpleActionState("move_base", MoveBaseAction, goal_cb=goal_cb, result_cb=navigation_result_callback,
                                                input_keys=["location", "goal"],
                                                outcomes = ["laser_scan", OUTCOME_FAILURE, "failed_go_home"],
                                                output_keys=["location", "goal"])
        smach.StateMachine.add("NAVIGATION_TO_GOAL", nav_state, 
                               transitions={"succeeded": "DONE", "aborted": OUTCOME_FAILURE, "preempted": OUTCOME_FAILURE, "laser_scan": "TAKING_LASER_SCAN", "failed_go_home": "NAVIGATION_TO_GOAL" })

        def monitor_cb(ud, msg):
            """This function is called when the Robot hears a scan from the /font/scan topic and saves the scan to the state machine user data"""

            ud.laser_scans.append(msg)

            # if it is the 1s scan, next destination is to rotate 180
            if len(ud.laser_scans) == 1:
                ud.goal = "A_180"
                return False

            # if it is the second scan, next destination is to go to the other point B
            if len(ud.laser_scans) == 2:
                ud.goal = "B"
                return False

            # if it is the third scan, next destination is to rotate 180
            if len(ud.laser_scans) == 3:
                ud.goal = "B_180"
                return False

            # if it is the fourth scan, next destination is home
            if len(ud.laser_scans) == 4:
                ud.goal = "HOME"
            return False

        laser_scan_state = smach_ros.MonitorState("/front/scan", LaserScan, monitor_cb, 
                                                input_keys=["laser_scans", "goal"],
                                                  output_keys=["goal", "laser_scans"])
        # state machine to print out laser scans
        smach.StateMachine.add("TAKING_LASER_SCAN", laser_scan_state,  transitions={'invalid':'NAVIGATION_TO_GOAL', 'valid':OUTCOME_FAILURE, 'preempted':OUTCOME_FAILURE} ) # smach.StateMachine.add("NAVIGATING_TO_GOAL", mb_state_one, transitions={"succeeded": "TAKING_LASER_SCAN_ONE", "aborted": "NAVIGATING_TO_HOME" })

        def done_cb(userdata):
            try:
                rospy.loginfo(f"NAVIGATION JOURNEY COMPLETE")
                for index, scan in enumerate(userdata.laser_scans):
                    rospy.loginfo(f"LASER SCAN{index}: {scan}")
                return OUTCOME_SUCCESS
            except Exception:
                return OUTCOME_FAILURE

        done_state = smach.CBState(done_cb, outcomes=[OUTCOME_FAILURE, OUTCOME_SUCCESS], input_keys=["laser_scans"])
        smach.StateMachine.add("DONE", done_state, transitions={OUTCOME_FAILURE: OUTCOME_FAILURE, OUTCOME_SUCCESS: OUTCOME_SUCCESS})

    sis = smach_ros.IntrospectionServer('navigation_smach', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
