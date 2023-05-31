#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_transformation import quaternion_from_euler
import ORB_SLAM3

class ImageGrabber:
    def __init__(self, pSLAM):
        self.mpSLAM = pSLAM

    def GrabImage(self, msg):
        # Copy the ROS image message to cv2 image.
        try:
            cv_image = cv_bridge.CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr('cv_bridge exception: %s', e)
            return

        # ORB-SLAM3 runs in TrackMonocular()
        Tcc0 = self.mpSLAM.TrackMonocular(cv_image, msg.header.stamp.to_sec())
        Twc = np.linalg.inv(Tcc0 @ Tc0w)

        msg_time = msg.header.stamp

        self.publish_ros_camera_pose(Twc, msg_time)
        self.publish_ros_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time)
        self.publish_ros_tracked_mappoints(self.mpSLAM.GetTrackedMapPoints(), msg_time)

    def publish_ros_camera_pose(self, Twc, msg_time):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg_time
        pose_msg.header.frame_id = world_frame_id

        # Extract translation
        pose_msg.pose.position.x = Twc[0, 3]
        pose_msg.pose.position.y = Twc[1, 3]
        pose_msg.pose.position.z = Twc[2, 3]

        # Extract rotation as quaternion
        q = quaternion_from_euler(*np.deg2rad([roll, pitch, yaw]))
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]

        camera_pose_publisher.publish(pose_msg)

    def publish_ros_tf_transform(self, Twc, parent_frame_id, child_frame_id, msg_time):
        transform_msg = TransformStamped()
        transform_msg.header.stamp = msg_time
        transform_msg.header.frame_id = parent_frame_id
        transform_msg.child_frame_id = child_frame_id

        # Extract translation
        transform_msg.transform.translation.x = Twc[0, 3]
        transform_msg.transform.translation.y = Twc[1, 3]
        transform_msg.transform.translation.z = Twc[2, 3]

        # Extract rotation as quaternion
        q = quaternion_from_euler(*np.deg2rad([roll, pitch, yaw]))
        transform_msg.transform.rotation.x = q[0]
        transform_msg.transform.rotation.y = q[1]
        transform_msg.transform.rotation.z = q[2]
        transform_msg.transform.rotation.w = q[3]

        tf_msg = TFMessage([transform_msg])
        tf_publisher.publish(tf_msg)

    def publish_ros_tracked_mappoints(self, mappoints, msg_time):
        # Publish tracked map points if available
        # Assuming mappoints is a list of map points
        pass


def main():
    rospy.init_node('Mono')
    rospy.loginfo('Mono node initialized.')

    if rospy.get_param('~voc_file', 'file_not_set') == 'file_not_set' or rospy.get_param('~settings_file', 'file_not_set') == 'file_not_set':
        rospy.logerr('Please provide voc_file and settings_file in the launch file')
        rospy.signal_shutdown('Missing parameters')
        return

    global world_frame_id, cam_frame_id, Tc0w, roll, pitch, yaw
    world_frame_id = rospy.get_param('~world_frame_id', 'map')
    cam_frame_id = rospy.get_param('~cam_frame_id', 'camera')

    # World frame orientation
    rpy_deg = np.zeros(3)
    angle_names = ['roll', 'pitch', 'yaw']
    for i, angle in enumerate(angle_names):
        rpy_deg[i] = rospy.get_param('~world_' + angle, 0)

    # Convert degrees to radians
    roll, pitch, yaw = np.deg2rad(rpy_deg)

    # Create SLAM system. It initializes all system threads and gets ready to process frames.
    sensor_type = ORB_SLAM3.System.MONOCULAR
    SLAM = ORB_SLAM3.System(voc_file, settings_file, sensor_type)
    igb = ImageGrabber(SLAM)

    sub_img0 = rospy.Subscriber('/camera/image_raw', Image, igb.GrabImage)

    rospy.spin()

    # Stop all threads
    SLAM.Shutdown()

    rospy.loginfo('Shutting down.')


if __name__ == '__main__':
    main()