#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
import actionlib
from move_base_msgs.msgs import MoveBaseAction, MoveBaseGoal


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

    sm = smach.StateMachine(outcomes=[OUTCOME_SUCCESS, OUTCOME_FAILURE])


    #userdata
    def goal_cb_1():
        goal_one = MoveBaseGoal()
        goal_one.target_pose.header.frame_id = "map"
        goal_one.target_pose.header.stamp = rospy.Time.now()
        goal_one.target_pose.pose.position.x = 0.5
        goal_one.target_pose.pose.orientation.w = 1.0
        return goal_one


    with sm:
        smach.StateMachine.add("NAVIGATION",
                               smach_ros.SimpleActionState("move_base",
                                                 MoveBaseAction,
                                                 goal_cb=goal_cb_1),
                               tratisions={"succeeded": OUTCOME_SUCCESS})


    sis = smach_ros.IntrospectionServer('navigation_smach', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
