#!/usr/bin/env python

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np

sensor_type = None
world_frame_id = None
cam_frame_id = None
imu_frame_id = None
Tc0w = None
pose_pub = None
map_points_pub = None

def setup_ros_publishers(node_handler, image_transport, rpy_rad):
    global pose_pub, map_points_pub, Tc0w, world_frame_id, cam_frame_id

    pose_pub = node_handler.advertise('orb_slam3/camera_pose', PoseStamped, queue_size=1)
    map_points_pub = node_handler.advertise('orb_slam3/map_points', PointCloud2, queue_size=1)

    if not np.allclose(rpy_rad, 0):
        roll, pitch, yaw = rpy_rad
        qRPY = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        Tc0w = tf.transformations.quaternion_matrix(qRPY)
        rospy.loginfo(f"World frame will be rotated by RPY (in that order) {roll} {pitch} {yaw} (rad)")

def publish_ros_camera_pose(Twc_SE3f, msg_time):
    global pose_pub, world_frame_id

    pose_msg = PoseStamped()
    pose_msg.header.frame_id = world_frame_id
    pose_msg.header.stamp = msg_time

    pose_msg.pose.position.x = Twc_SE3f.translation()[0]
    pose_msg.pose.position.y = Twc_SE3f.translation()[1]
    pose_msg.pose.position.z = Twc_SE3f.translation()[2]

    pose_msg.pose.orientation.w = Twc_SE3f.unit_quaternion().coeffs()[0]
    pose_msg.pose.orientation.x = Twc_SE3f.unit_quaternion().coeffs()[1]
    pose_msg.pose.orientation.y = Twc_SE3f.unit_quaternion().coeffs()[2]
    pose_msg.pose.orientation.z = Twc_SE3f.unit_quaternion().coeffs()[3]

    pose_pub.publish(pose_msg)

def publish_ros_tf_transform(Twc_SE3f, frame_id, child_frame_id, msg_time):
    global world_frame_id

    transform = tf.TransformBroadcaster()
    transform.sendTransform(
        (Twc_SE3f.translation()[0], Twc_SE3f.translation()[1], Twc_SE3f.translation()[2]),
        (Twc_SE3f.unit_quaternion().coeffs()[1], Twc_SE3f.unit_quaternion().coeffs()[2],
         Twc_SE3f.unit_quaternion().coeffs()[3], Twc_SE3f.unit_quaternion().coeffs()[0]),
        msg_time,
        child_frame_id,
        frame_id
    )

def publish_ros_tracked_mappoints(map_points, msg_time):
    global map_points_pub, world_frame_id

    if len(map_points) == 0:
        rospy.logwarn("Map point vector is empty!")
        return

    cloud = PointCloud2()
    cloud.header.stamp = msg_time
    cloud.header.frame_id = world_frame_id
    cloud.height = 1
    cloud.width = len(map_points)
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    cloud.data = []

    for map_point in map_points:
        if map_point is not None:
            pMPw = map_point.GetWorldPos()

            if sensor_type == ORB_SLAM3.System.MONOCULAR or sensor_type == ORB_SLAM3.System.STEREO:
                Tc0mp = np.eye(4)
                Tc0mp[:3, 3] = pMPw
                Twmp = np.linalg.inv(Tc0w) @ Tc0mp
                pMPw = Twmp[:3, 3]

            cloud.data.extend([pMPw[0].astype(np.float32).tobytes(),
                              pMPw[1].astype(np.float32).tobytes(),
                              pMPw[2].astype(np.float32).tobytes()])

    map_points_pub.publish(cloud)

def tracked_mappoints_to_pointcloud(map_points, msg_time):
    global world_frame_id

    if len(map_points) == 0:
        rospy.logwarn("Map point vector is empty!")
        return None

    num_channels = 3
    cloud = PointCloud2()
    cloud.header.stamp = msg_time
    cloud.header.frame_id = world_frame_id
    cloud.height = 1
    cloud.width = len(map_points)
    cloud.is_bigendian = False
    cloud.is_dense = True
    cloud.point_step = num_channels * 4
    cloud.row_step = cloud.point_step * cloud.width
    cloud.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    cloud.data = []

    for i, map_point in enumerate(map_points):
        if map_point is not None:
            pMPw = map_point.GetWorldPos()

            if sensor_type == ORB_SLAM3.System.MONOCULAR or sensor_type == ORB_SLAM3.System.STEREO:
                Tc0mp = np.eye(4)
                Tc0mp[:3, 3] = pMPw
                Twmp = np.linalg.inv(Tc0w) @ Tc0mp
                pMPw = Twmp[:3, 3]

            cloud.data.extend([pMPw[0].astype(np.float32).tobytes(),
                              pMPw[1].astype(np.float32).tobytes(),
                              pMPw[2].astype(np.float32).tobytes()])

    return cloud

def SE3f_to_tfTransform(T_SE3f):
    R_mat = T_SE3f.rotationMatrix()
    t_vec = T_SE3f.translation()

    R_tf = tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_matrix(R_mat))
    t_tf = tf.transformations.translation_from_matrix(T_SE3f.matrix())

    return tf.transformations.concatenate_matrices(t_tf, R_tf)

