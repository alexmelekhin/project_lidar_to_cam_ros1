#!/usr/bin/env python3

from typing import Tuple

import cv2
import matplotlib
import matplotlib.pyplot as plt
import message_filters
import numpy as np
import rospy
import tf
from cv_bridge import CvBridge
from ros_numpy.point_cloud2 import pointcloud2_to_xyz_array
from sensor_msgs.msg import CameraInfo, CompressedImage, PointCloud2

# using non-GUI backend solves OOM issue and fasten the processing
matplotlib.use("Agg")


def project_points_to_camera(
    points: np.ndarray, proj_matrix: np.ndarray, cam_res: Tuple[int, int]
) -> Tuple[np.ndarray, np.ndarray]:
    if points.shape[0] == 3:
        points = np.vstack((points, np.ones((1, points.shape[1]))))
    if len(points.shape) != 2 or points.shape[0] != 4:
        raise ValueError(
            f"Wrong shape of points array: {points.shape}; expected: (4, n), where n - number of points."
        )
    if proj_matrix.shape != (3, 4):
        raise ValueError(f"Wrong proj_matrix shape: {proj_matrix}; expected: (3, 4).")
    in_image = points[2, :] > 0
    depths = points[2, in_image]
    uvw = np.dot(proj_matrix, points[:, in_image])
    uv = uvw[:2, :]
    w = uvw[2, :]
    uv[0, :] /= w
    uv[1, :] /= w
    in_image = (uv[0, :] >= 0) * (uv[0, :] < cam_res[0]) * (uv[1, :] >= 0) * (uv[1, :] < cam_res[1])
    return uv[:, in_image].astype(int), depths[in_image]


def depths_to_colors(depths: np.ndarray, max_depth: int = 100, cmap: str = "hsv") -> np.ndarray:
    depths /= max_depth
    to_colormap = plt.get_cmap(cmap)
    rgba_values = to_colormap(depths, bytes=True)
    return rgba_values[:, :3].astype(int)


class LidarProjectionNode:
    max_lidar_distance = 100

    def __init__(self) -> None:
        rospy.init_node("lidar_projection_node", log_level=rospy.DEBUG)

        self.br = CvBridge()

        self.sub_camera_info = message_filters.Subscriber("camera_info", CameraInfo)
        self.sub_image = message_filters.Subscriber("image", CompressedImage)
        self.sub_lidar = message_filters.Subscriber("lidar", PointCloud2)

        self.pub_image = rospy.Publisher("pub_image", CompressedImage, queue_size=1)

        self.tf_listener = tf.TransformListener()

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_camera_info, self.sub_image, self.sub_lidar],
            queue_size=10,
            slop=0.05,
        )
        self.ts.registerCallback(self.on_image)

        rospy.loginfo("lidar_projection_node is ready")

    def on_image(
        self,
        camera_info: CameraInfo,
        compressed_image_msg: CompressedImage,
        lidar_msg: PointCloud2,
    ) -> None:
        rospy.logdebug("Received data")

        im_frame = compressed_image_msg.header.frame_id
        lidar_frame = lidar_msg.header.frame_id
        try:
            (trans, rot) = self.tf_listener.lookupTransform(im_frame, lidar_frame, rospy.Time(0))
            rospy.logdebug(f"Received transform from {lidar_frame} to {im_frame}: {trans, rot}")
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ):
            rospy.logerr(f"lookupTransform from {lidar_frame} to {im_frame} failed")
            return

        tf_lidar2cam = tf.transformations.quaternion_matrix(rot)
        tf_lidar2cam[:3, 3] = trans

        proj_cam = np.array(camera_info.P).reshape((3, 4))
        cam_res = (camera_info.width, camera_info.height)

        im_ts = compressed_image_msg.header.stamp.to_nsec()
        pc_ts = lidar_msg.header.stamp.to_nsec()
        ts_diff = np.abs(im_ts - pc_ts) / 1000000
        rospy.logdebug(f"ts_diff = {ts_diff:.3f} ms")

        image = self.br.compressed_imgmsg_to_cv2(compressed_image_msg)

        lidar_points = pointcloud2_to_xyz_array(lidar_msg, remove_nans=True).T
        if lidar_points.shape[0] == 3:
            lidar_points = np.vstack((lidar_points, np.ones((1, lidar_points.shape[1]))))

        lidar_points = tf_lidar2cam @ lidar_points

        uv, depth = project_points_to_camera(lidar_points, proj_cam, cam_res)
        rgb_distances = depths_to_colors(depth, max_depth=self.max_lidar_distance)
        for point, d in zip(uv.T, rgb_distances):
            c = (int(d[0]), int(d[1]), int(d[2]))
            cv2.circle(image, point, radius=2, color=c, thickness=cv2.FILLED)

        new_msg = self.br.cv2_to_compressed_imgmsg(image)
        self.pub_image.publish(new_msg)
        rospy.logdebug("Published image")


if __name__ == "__main__":
    lidar_projection_node = LidarProjectionNode()
    rospy.spin()
