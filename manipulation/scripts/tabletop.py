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


def get_tf_homogeneous_transform(parent_link: str, child_link: str) -> np.ndarray:
    """
    COMPLETE: No need to implement
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


def link1_T_link2(joint_value: float) -> np.ndarray:
    pass


def link2_T_link3(joint_value: float) -> np.ndarray:
    pass


def link3_T_link4(joint_value: float) -> np.ndarray:
    pass


def link4_T_link5(joint_value: float) -> np.ndarray:
    pass


def link5_T_link6(joint_value: float) -> np.ndarray:
    pass


def link6_T_link7(joint_value: float) -> np.ndarray:
    pass


def link7_T_ee() -> np.ndarray:
    """
    This function returns the static transform between the seventh link and the end effector.
    It is static because there are no joints between these frames.
    You can hard code this if you like or get it from tf.

    Returns:
        np.ndarray: _description_
    """


def link0_T_ee(joint_values: List[float]) -> np.ndarray:
    """
    This function returns the full forward kinematics from base to end effector

    Args:
        joint_values (List[float]): The seven joint angles

    Returns:
        np.ndarray: The homogenous tranformation between base and end effector
    """


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
        forward_kinematics_transform = forward_kinematics(joint_state.position[i])

        error = np.abs(tf_transform - forward_kinematics_transform)
        if np.any(error > 0.001):
            rospy.logerr(f'The matrices for link {i} to link {i+1} do not match.\n'
                         f'{tf_transform}\n{forward_kinematics_transform}')
    
    # test the full forward kinematics
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

