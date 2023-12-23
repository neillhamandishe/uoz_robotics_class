#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


OUTCOME_SUCCESS = "success"
OUTCOME_FAILURE = "failed"


class WaitingForGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["goal_not_found", "goal_found", OUTCOME_SUCCESS])
        self.count = 0
        self.subscriber = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.execute)

    def execute(self, userdata=None):
        print(f"data: {userdata}")
        self.count += 1
        if self.count < 3:
            return "goal_found"
        return OUTCOME_SUCCESS


class GettingPath(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["path_not_found", "path_found"])

    def execute(self, userdata):
        return "path_found"


class ExecutingPath(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=[OUTCOME_FAILURE, "execution_succeeded"])

    def execute(self, userdata):
        return "execution_succeeded"


def main():
    rospy.init_node("navigation_smach")

    sm = smach.StateMachine(outcomes=[OUTCOME_SUCCESS, OUTCOME_FAILURE, "aborted", "preempted"])
    sm.userdata.goal = None
    sm.userdata.path = None

    
    with sm:
        pose = Pose(
            position = Point(
            x=1.009905219078064,
            y=1.1159578561782837,
            z=0.0),
            orientation = Quaternion(x=0, y=0, z=0, w=1)
        )
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.pose = pose
        nav_goal.target_pose.header.frame_id = "map"
        
        move_base_state = smach_ros.SimpleActionState('move_base', MoveBaseAction,
            goal=nav_goal, exec_timeout=rospy.Duration(60.0))
        smach.StateMachine.add("NAVIGATING_TO_ONE", move_base_state, transitions={"succeeded": "success", "aborted": "failed" })

        # smach.StateMachine.add("WAIT_FOR_GOAL",
        #                        smach_ros.MonitorState("/move_base_simple/goal",
        #                                          PoseStamped,
        #                                          goal_cb,
        #                                 output_keys=['goal']
        #                             ),
        #                         transitions={
        #                             'invalid': 'NAVIGATE_TO_GOAL',
        #                             'valid': 'WAIT_FOR_GOAL',
        #                             'preempted': 'preempted'
        #                         })

    sis = smach_ros.IntrospectionServer('navigation_smach', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
