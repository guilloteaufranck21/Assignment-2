import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import apriltag

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge


class TagCamera(Node):
    def __init__(self):
        super().__init__('tagcamera_node')

        # OpenCV bridge for a ros image to opencv format
        self.bridge = CvBridge()

        # AprilTag detector to scan white patts.
        self.detector = apriltag.Detector()

        #  declaration of camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_frame = None

        # Tag size(m)
        self.tag_size = 0.05

        # Subscription to camera imfo and camera
        self.create_subscription(CameraInfo,'/rgb_camera/camera_info',self.camera_info_cb,10)
        self.create_subscription(Image,'/rgb_camera/image',self.image_cb,10)

        # Publishing
        self.pose_pub = self.create_publisher(PoseStamped,'/apriltag/pose_camera',10)

        self.get_logger().info('AprilTag node started')

    def camera_info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_frame = "external_camera/link/rgb_camera" #the frame
            self.get_logger().info(f'Camera intrinsics received from frame: {self.camera_frame}')

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return  # wait for data in camera info

        # Convert ROS Image to OpenCV "mono8 because black-white"
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')

        # Detect AprilTags
        detections = self.detector.detect(gray)

        for det in detections:
            pose = self.compute_pose(det)
            if pose:
                self.pose_pub.publish(pose)

    def compute_pose(self, det):
        # 3D points of tag corners in tag frame
        s = self.tag_size / 2.0
        object_points = np.array([
            [-s, -s, 0.0],
            [ s, -s, 0.0],
            [ s,  s, 0.0],
            [-s,  s, 0.0]
        ], dtype=np.float32)

        image_points = det.corners.astype(np.float32)

        #PnP tvec for translational vec and rvec for rotational to compute distance & rotation of april tags
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            self.get_logger().warn('PnP failed')
            return None

        # Convert rotation vector to quaternion
        R, _ = cv2.Rodrigues(rvec)
        q = self.rotation_matrix_to_quaternion(R)

        # Fill PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.camera_frame

        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.get_logger().info(f'Tag {det.tag_id} detected at {tvec.flatten()}')

        return pose

    def rotation_matrix_to_quaternion(self,R):
        # Conversion of rotation matrix to quaternion
        qw = np.sqrt(1.0 + np.trace(R)) / 2.0
        qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
        qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
        qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = TagCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
