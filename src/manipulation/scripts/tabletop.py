#!/usr/bin/env python3

from typing import List
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf.transformations
import PyKDL
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
import functools
import tf2_ros
from sensor_msgs.msg import JointState
from ros_numpy import numpify


@functools.lru_cache
def get_kdl_chain():
    robot = URDF.from_parameter_server()
    kin = KDLKinematics(robot, 'panda_link0', 'panda_link7')
    return kin


@functools.lru_cache
def get_tf_buffer():
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    return tf_buffer


def transform_message_to_homogeneous_transform(transform_msg: TransformStamped) -> np.ndarray:
    """
    This function should convert a ROS TF message to a 4x4 numpy matrix.
    Please review the functions in tf.transformations for an easy way to do this.

    Args:
        transform_msg (TFMessage): _description_

    Returns:
        np.ndarray: _description_
    """
    transform = transform_msg.transform

    translation = (transform.translation.x, transform.translation.y, transform.translation.z)
    rotation = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)

    translation_matrix = tf.transformations.translation_matrix(translation)
    rotation_matrix = tf.transformations.quaternion_matrix(rotation)

    trans = tf.transformations.concatenate_matrices(translation_matrix, rotation_matrix)

    return trans


def get_tf_homogeneous_transform(parent_link: str, child_link: str) -> np.ndarray:
    """
    C/home/neill/ros/uoz_robotics_class/src/hrl-kdl/pykdl_utils/src/pykdl_utils/kdl_kinematicsopyOMPLETE: No need to implement
    This function gets the transform between parent and child link directly from
    TF and puts it into homogenous transform format

    Args:
        parent_link_idx (str): _description_
        child_link_idx (str): _description_

    Returns:
        np.ndarray: The homogeneous transform representation between parent and child
    """
    tf_buffer = get_tf_buffer()
    transform_message = tf_buffer.lookup_transform(f'panda_{parent_link}',
                                                   f'panda_{child_link}',
                                                   rospy.Time(),
                                                   timeout=rospy.Duration(1))
    return transform_message_to_homogeneous_transform(transform_message)


def kdl_frame_to_homogeneous_transform(kdl_frame: PyKDL.Frame) -> np.ndarray:
    """
    COMPLETE: No need to implement
    This function takes a transform defined in the KDL library and converts it
    to a 4x4 homogenous transform defined in numpy.
    Args:
        kdlf_frame (PyKDL.Frame): A transform defined as a KDL Frame

    Returns:
        np.ndarray: The same transform defined as a numpy 4x4 matrix
    """
    matrix = tf.transformations.quaternion_matrix(kdl_frame.M.GetQuaternion())
    matrix[:3, 3] = list(kdl_frame.p)
    return matrix


def get_zero_angle_transform_between_links(parent_link_idx: float) -> np.ndarray:
    """
    COMPLETE: No need to implement
    This function tells you the transform between the frames at link N and link N+1
    when the joint N+1 angle is zero.

    You can use this to split the transform from N_T_N+1(theta) into two parts:
        - N       _T_ N+1(0)
        - N+1(0)  _T_ N+1(theta)

    This function returns N_T_N+1(0)

    Args:
        parent_link_idx (float): N

    Returns:
        np.ndarray: The 4x4 homogeneous transform for N->N+1(0)
    """
    kin = get_kdl_chain()
    frame = kin.chain.getSegment(parent_link_idx).getFrameToTip()
    return kdl_frame_to_homogeneous_transform(frame)


def get_rotation_axis(joint_index: float):
    """
    COMPLETE: No need to implement
    This function tells you the axis around which the joint rotates.
    For example, if it rotates around Z it will return [0, 0, 1].
    You can also look at the frames in TF and see the axis it rotates around. 

    Args:
        joint_index (float): _description_

    Returns:
        _type_: _description_
    """
    kin = get_kdl_chain()
    return np.array(kin.chain.getSegment(joint_index).getJoint().JointAxis())


def get_rotation_about_axis(angle, axis=(0, 0, 1)) -> np.ndarray:
    """
    This function tells you the transform for a frame that is rotated by some angle
    about one of it's axes. The default is the Z axis (0, 0, 1) since all joints in
    this robot rotate about the Z axis.

    You can use this to find N+1(0)_T_N+1(theta)

    To implement, look at tf.transformations.rotation_matrix.

    Args:
        angle (_type_): _description_
        axis (tuple, optional): _description_. Defaults to (0, 0, 1).

    Returns:
        np.ndarray: _description_
    """
    rotation_matrix = tf.transformations.rotation_matrix(angle, axis)
    return rotation_matrix


def get_transform(N: int, joint_value: float) -> np.ndarray:
    """This helper function returns transform N T N+1(joint_value)

    Args:
        N (int): The origin frame
        joint_value (float): The angle theta of joint at frame N+1

    Returns:
        np.ndarray: The 4x4 homogeneous transform matrix of N T N+1(theta)
    """
    # N => N+1(0)
    t1 = get_zero_angle_transform_between_links(N)
    #axis = get_rotation_axis(N)
    #axis_tuple = tuple(axis.tolist())

    #print(f"axis {axis}")
    #print(f"axis_tuple {axis_tuple}")
    t2 = get_rotation_about_axis(joint_value)
    return tf.transformations.concatenate_matrices(t1, t2)


def link0_T_link1(joint_value: float) -> np.ndarray:
    """
    This function should take in the joint 1 value and produce a 4x4
    homogeneous transformation matrix that describes the translation
    from the base link to link1

    Args:
        joint_value (float): The angle in radians of joint 1

    Returns:
        np.ndarray: The 4x4 homogeneous transformation matrix that
            describes the translation from the base link to link1
    """
    #0 to 1(0)
    return get_transform(0, joint_value)


def link1_T_link2(joint_value: float) -> np.ndarray:
    """print(f"joint_value {joint_value}")
    t1 = get_zero_angle_transform_between_links(1)
    axis = get_rotation_axis(1)
    axis_tuple = tuple(axis.tolist())

    print(f"axis {axis}")
    print(f"axis_tuple {axis_tuple}")
    t2 = get_rotation_about_axis(joint_value, (0, 0, 1))
    return  np.dot(t1, t2)"""
    return get_transform(1, joint_value)


def link2_T_link3(joint_value: float) -> np.ndarray:
    return get_transform(2, joint_value)


def link3_T_link4(joint_value: float) -> np.ndarray:
    return get_transform(3, joint_value)


def link4_T_link5(joint_value: float) -> np.ndarray:
    return get_transform(4, joint_value)


def link5_T_link6(joint_value: float) -> np.ndarray:
    return get_transform(5, joint_value)


def link6_T_link7(joint_value: float) -> np.ndarray:
    return get_transform(6, joint_value)


def link7_T_ee() -> np.ndarray:
    """
    This function returns the static transform between the seventh link and the end effector.
    It is static because there are no joints between these frames.
    You can hard code this if you like or get it from tf.

    Returns:
        np.ndarray: _description_
    """
    return get_tf_homogeneous_transform("link7", "EE")


def link0_T_ee(joint_values: List[float]) -> np.ndarray:
    """
    This function returns the full forward kinematics from base to end effector

    Args:
        joint_values (List[float]): The seven joint angles

    Returns:
        np.ndarray: The homogenous tranformation between base and end effector
    """
    """forward_kinematics_funcs = [
        link0_T_link1,
        link1_T_link2,
        link2_T_link3,
        link3_T_link4,
        link4_T_link5,
        link5_T_link6,
        link6_T_link7,
    ]
    forward_kin = None
    index = 0
    while index < 7:
        value = joint_values[index]
        kin_at_index = forward_kinematics_funcs[index](value)
        forward_kin = kin_at_index if forward_kin is None else np.dot(forward_kin, kin_at_index)
        index += 1
    return forward_kin"""
    l0_T_l1 = link0_T_link1(joint_values[0])

    # homogeneous transformation matrix from link1 to link2
    l1_T_l2 = link1_T_link2(joint_values[1])

    # homogeneous transformation matrix from link2 to link3
    l2_T_l3 = link2_T_link3(joint_values[2])

    # homogeneous transformation matrix from link3 to link4
    l3_T_l4 = link3_T_link4(joint_values[3])

    # homogeneous transformation matrix from link4 to link5
    l4_T_l5 = link4_T_link5(joint_values[4])

    # homogeneous transformation matrix from link5 to link6
    l5_T_l6 = link5_T_link6(joint_values[5])

    # homogeneous transformation matrix from link6 to link7
    l6_T_l7 = link6_T_link7(joint_values[6])

    # homogeneous transformation matrix from link7 to the end effector
    l7_T_ee_transform = link7_T_ee()

    # the overall homogeneous transformation matrix from link0 to the end effector
    #link0_T_ee_transform = np.dot(l0_T_l1, np.dot(l1_T_l2, np.dot(l2_T_l3, np.dot(l3_T_l4, np.dot(l4_T_l5, np.dot(l5_T_l6, np.dot(l6_T_l7, l7_T_ee_transform)))))))


    link0_T_2 = np.dot(l0_T_l1, l1_T_l2)
    link2_T_4 = np.dot(l2_T_l3, l3_T_l4)
    link0_T_4 = np.dot(link0_T_2, link2_T_4)

    link4_T_6 = np.dot(l4_T_l5, l5_T_l6)
    link6_T_ee = np.dot(l6_T_l7, l7_T_ee_transform)
    link6_T_ee = np.dot(link4_T_6, link6_T_ee)

    return np.dot(link0_T_4, link6_T_ee)

def jacobian(joint_values: List[float]) -> np.ndarray:
    """
    COMPLETE: No need to implement

    Args:
        joint_values (List[float]): The seven joint angles

    Returns:
        np.ndarray: _description_
    """
    kin = get_kdl_chain()
    return kin.jacobian(joint_values)


def inverse_kinematics(joint_values: List[float], target_pose: TransformStamped) -> List[float]:
    """
    This function takes the current joint values and a target EE pose and calculates joint angles
    that will give us that EE pose

    Args:
        joint_values (List[float]): The seven joint angles
        target_pose (TransformStamped): Where we want the EE to be

    Returns:
        List[float]: The joint angles that produce the EE in the target pose
    """
    return []


if __name__ == '__main__':
    # initialize the node
    rospy.init_node('tabletop')

    forward_kinematics_funcs = [
        link0_T_link1,
        link1_T_link2,
        link2_T_link3,
        link3_T_link4,
        link4_T_link5,
        link5_T_link6,
        link6_T_link7,
    ]

    # get the current joint angles
    joint_state = rospy.wait_for_message('/joint_states', JointState)

    # test each forward kinematic function against tf
    for i, forward_kinematics in enumerate(forward_kinematics_funcs):
        tf_transform = get_tf_homogeneous_transform(f'link{i}', f'link{i+1}')
        print("here")
        forward_kinematics_transform = forward_kinematics(joint_state.position[i])

        error = np.abs(tf_transform - forward_kinematics_transform)
        if np.any(error > 0.001):
            rospy.logerr(f'The matrices for link {i} to link {i+1} do not match.\n'
                         f'{tf_transform}\n{forward_kinematics_transform}')

    # test the full forward kinematics
    print(f"forward kinematics for link 7 to ee")
    expected7_T_EE = get_tf_homogeneous_transform("link7", "EE")
    print(f"expected: {expected7_T_EE}")
    got7_T_EE = link7_T_ee()
    print(f"got: {got7_T_EE}")



    tf_transform = get_tf_homogeneous_transform('link0', 'EE')
    forward_kinematics_transform = link0_T_ee(joint_state.position)

    error = np.abs(tf_transform - forward_kinematics_transform)
    if np.any(error > 0.001):
        rospy.logerr(f'The matrices for the full forward kinematics do not match.\n'
                        f'{tf_transform}\n{forward_kinematics_transform}')

    # run the inverse kinematics
    target_transform = TransformStamped()
    target_transform.transform.translation.x = 0.569
    target_transform.transform.translation.y = 0.211
    target_transform.transform.translation.z = 0.259
    target_transform.transform.rotation.x = -1
    target_transform.transform.rotation.y = 0
    target_transform.transform.rotation.z = 0
    target_transform.transform.rotation.w = 0

    target_pose = transform_message_to_homogeneous_transform(target_transform)

    rospy.loginfo(inverse_kinematics(joint_state, target_pose))

