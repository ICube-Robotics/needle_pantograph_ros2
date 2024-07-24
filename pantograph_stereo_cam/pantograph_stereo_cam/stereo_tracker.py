#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
import rclpy.time
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import os
import csv


class StereoTracker(Node):
    def __init__(self):
        super().__init__('stereo_tracker')
        # Img publisher
        # self.img_1_publisher = self.create_publisher(Image, 'usb_cam_1/image_raw', 10)
        # self.img_2_publisher = self.create_publisher(Image, 'usb_cam_2/image_raw', 10)

        # tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # tf publisher
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Video capture
        # Check device index with "v4l2-ctl --list-devices"
        self.cap_1 = cv2.VideoCapture(4)  # Change the device index as needed
        self.cap_1.set(cv2.CAP_PROP_FPS, 30)

        self.cap_2 = cv2.VideoCapture(2)  # Change the device index as needed
        self.cap_2.set(cv2.CAP_PROP_FPS, 30)

        # Read stereo system params from yaml files
        dirname = os.path.dirname(__file__)
        # Camera 1
        cam_1_params_path = os.path.join(dirname, '../config/world_to_cam_0_info.yaml')

        with open(cam_1_params_path) as f1:
            data_1 = yaml.safe_load(f1)

        self.cam_1_intrinsic_mat = np.array(data_1['camera_matrix']['data']).reshape(3, 3)
        self.cam_1_distorsion_coeffs = np.array(data_1['distortion_coefficients']['data']).reshape(-1, 5)
        self.cam_1_projection_mat = np.array(data_1['projection_matrix']['data']).reshape(3, 4)
        self.img_size = [data_1['image_width'], data_1['image_height']]

        # Camera 2
        cam_2_params_path = os.path.join(dirname, '../config/world_to_cam_1_info.yaml')

        with open(cam_2_params_path) as f2:
            data_2 = yaml.safe_load(f2)

        self.cam_2_intrinsic_mat = np.array(data_2['camera_matrix']['data']).reshape(3, 3)
        self.cam_2_distorsion_coeffs = np.array(data_2['distortion_coefficients']['data']).reshape(-1, 5)
        self.cam_2_projection_mat = np.array(data_2['projection_matrix']['data']).reshape(3, 4)

        # Create Trackbars for detection adjustment
        cv2.namedWindow('Trackbars')
        cv2.createTrackbar('Lower H', 'Trackbars', 70, 179, self.nothing)
        cv2.createTrackbar('Lower S', 'Trackbars', 82, 255, self.nothing)
        cv2.createTrackbar('Lower V', 'Trackbars', 100, 255, self.nothing)
        cv2.createTrackbar('Upper H', 'Trackbars', 92, 179, self.nothing)
        cv2.createTrackbar('Upper S', 'Trackbars', 255, 255, self.nothing)
        cv2.createTrackbar('Upper V', 'Trackbars', 255, 255, self.nothing)
        cv2.createTrackbar('Min Area', 'Trackbars', 200, 1500, self.nothing)
        cv2.createTrackbar('Max Area', 'Trackbars', 2000, 3000, self.nothing)
        cv2.createTrackbar('Circularity', 'Trackbars', 50, 100, self.nothing)
        cv2.createTrackbar('Convexity', 'Trackbars', 50, 100, self.nothing)
        cv2.createTrackbar('Inertia', 'Trackbars', 50, 100, self.nothing)

        # Initialization
        self.points_3D = [0, 0, 0]
        self.points_3D_world = [0, 0, 0]
        self.recorded_pts = []

    def timer_callback(self):
        ret_cam_1, frame_cam_1 = self.cap_1.read()
        ret_cam_2, frame_cam_2 = self.cap_2.read()

        # If frames are read correctly
        if ret_cam_1 and ret_cam_2:
            # ==================================================
            # Marker extraction
            # ==================================================

            # Detect marker in the left and right frames
            mask_1, coords_cam_1, area_1 = self.detect_marker(frame_cam_1)
            mask_2, coords_cam_2, area_2 = self.detect_marker(frame_cam_2)
            # print("Size of largest blob in cam_1 : ", area_1)
            # print("Size of largest blob in cam_2 : ", area_2)

            # ==================================================
            # Marker position triangulation
            # ==================================================
            if coords_cam_1 and coords_cam_2:
                self.points_3D, self.points_3D_world = self.get_marker_3D_coords(coords_cam_1, coords_cam_2)

            # ==================================================
            # Publish data
            # ==================================================

            self.node_handle(self.points_3D_world)

            # ==================================================
            # Display images
            # ==================================================

            # Blue point at the center of the detected blobs and save coordinates for display
            if coords_cam_1:
                cv2.circle(frame_cam_1, coords_cam_1, 5, (255, 0, 0), -1)
                coord_text_1 = f"Marker coords in image 1 : ({coords_cam_1[0]}, {coords_cam_1[1]})"
            else:
                coord_text_1 = "Marker coords in image 1 : (unknown,unknown)"

            if coords_cam_2:
                cv2.circle(frame_cam_2, coords_cam_2, 5, (255, 0, 0), -1)
                coord_text_2 = f"Marker coords in image 2 : ({coords_cam_2[0]}, {coords_cam_2[1]})"
            else:
                coord_text_2 = "Marker coords in image 2 : (unknown,unknown)"

            # Set window position and size
            window_height = 960
            window_width = 1280
            window_name = 'pantograph_stereo_cam'
            # cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.moveWindow(window_name, 100, 20)
            cv2.resizeWindow(window_name, window_width, window_height)
            img_height = window_height // 2
            img_width = (window_width - 50) // 2

            # Resize images for display
            frame_1_resized = cv2.resize(frame_cam_1, (img_width, img_height))
            frame_2_resized = cv2.resize(frame_cam_2, (img_width, img_height))

            mask_1_resized = cv2.resize(mask_1, (img_width, img_height))
            mask_2_resized = cv2.resize(mask_2, (img_width, img_height))

            height, width, _ = frame_1_resized.shape
            combined_height = height * 2 + 50  # Extra space for legends
            combined_width = width * 2

            combined_frame = np.zeros((combined_height, combined_width, 3), dtype=np.uint8)
            combined_frame[0:height, 0:width] = frame_1_resized
            combined_frame[0:height, width:width * 2] = frame_2_resized

            # Convert masks to BGR for display
            mask_1_bgr = cv2.cvtColor(mask_1_resized, cv2.COLOR_GRAY2BGR)
            mask_2_bgr = cv2.cvtColor(mask_2_resized, cv2.COLOR_GRAY2BGR)

            combined_frame[height:height * 2, 0:width] = mask_1_bgr
            combined_frame[height:height * 2, width:width * 2] = mask_2_bgr

            # Add legends
            cv2.putText(combined_frame, 'Camera 1 image', (width // 4, combined_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(combined_frame, 'Camera 2 image', (width + width // 4, combined_height - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

            # Add text with coordinates in image frame
            cv2.putText(combined_frame, coord_text_1, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(combined_frame, coord_text_2, (width + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)

            # Add 3D coordinates legend
            coord_text_3d = f"3D coords in Chessboard frame (x, y, z): ({self.points_3D[0]:.2f}, {self.points_3D[1]:.2f}, {self.points_3D[2]:.2f})"
            coord_text_3d_world = f"Test 3D coords in world frame (x, y, z): ({self.points_3D_world[0]:.2f}, {self.points_3D_world[1]:.2f}, {self.points_3D_world[2]:.2f})"
            cv2.putText(combined_frame, coord_text_3d, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(combined_frame, coord_text_3d_world, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2, cv2.LINE_AA)

            # Display in window
            k = cv2.waitKey(1)
            cv2.imshow(window_name, combined_frame)

            if k == 27:
                # if ESC is pressed at any time, the program will exit.
                quit()

            if k == 32:
                # if SPACE is pressed record point coordinates
                self.recorded_pts.append(self.points_3D_world)
                print('saving pt : ', self.points_3D_world)
                if len(self.recorded_pts) == 30:
                    self.save_recorded_pts()  # save points to csv file
                    self.recorded_pts = []

    def detect_marker(self, frame):
        # Invert colors (cyan is easier to detect than red)
        frame_inv = ~frame
        # Convert to HSV color space
        frame_inv_hsv = cv2.cvtColor(frame_inv, cv2.COLOR_BGR2HSV)

        # Get the current positions of the trackbars
        lower_h = cv2.getTrackbarPos('Lower H', 'Trackbars')
        lower_s = cv2.getTrackbarPos('Lower S', 'Trackbars')
        lower_v = cv2.getTrackbarPos('Lower V', 'Trackbars')
        upper_h = cv2.getTrackbarPos('Upper H', 'Trackbars')
        upper_s = cv2.getTrackbarPos('Upper S', 'Trackbars')
        upper_v = cv2.getTrackbarPos('Upper V', 'Trackbars')

        # Parameters for SimpleBlobDetector
        detector_params = cv2.SimpleBlobDetector_Params()
        detector_params.filterByColor = True
        detector_params.blobColor = 255
        detector_params.filterByArea = True
        detector_params.minArea = cv2.getTrackbarPos('Min Area', 'Trackbars')
        detector_params.maxArea = cv2.getTrackbarPos('Max Area', 'Trackbars')
        detector_params.filterByCircularity = True
        detector_params.minCircularity = cv2.getTrackbarPos('Circularity', 'Trackbars')/100  # 0.67
        detector_params.filterByConvexity = True
        detector_params.minConvexity = cv2.getTrackbarPos('Convexity', 'Trackbars')/100  # 0.87
        detector_params.filterByInertia = False
        detector_params.minInertiaRatio = cv2.getTrackbarPos('Inertia', 'Trackbars')/100  # 0.01

        self.detector = cv2.SimpleBlobDetector_create(detector_params)

        # Define the cyan color range in HSV
        lower_cyan = np.array([lower_h, lower_s, lower_v])
        upper_cyan = np.array([upper_h, upper_s, upper_v])

        # Creat mask
        mask = cv2.inRange(frame_inv_hsv, lower_cyan, upper_cyan)

        # Apply Morphological operations to remove noise and fill gaps
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        # Detect blobs
        keypoints = self.detector.detect(mask)

        # Find and return the coords of the largest blob (To further filter noise in the image)
        if keypoints:
            largest_blob = max(keypoints, key=lambda k: k.size)
            diameter = largest_blob.size
            blob_area = np.pi * (diameter/2) ** 2
            coords = [int(largest_blob.pt[0]), int(largest_blob.pt[1])]
            return mask, coords, blob_area
        else:
            return mask, None, None

    def get_marker_3D_coords(self, coords_cam_1, coords_cam_2):
        # Initialization
        undistorted_pts_cam_1 = [0, 0]
        undistorted_pts_cam_2 = [0, 0]
        K_1 = self.cam_1_intrinsic_mat
        d_1 = self.cam_1_distorsion_coeffs
        P_1 = self.cam_1_projection_mat
        K_2 = self.cam_2_intrinsic_mat
        d_2 = self.cam_2_distorsion_coeffs
        P_2 = self.cam_2_projection_mat

        # Convert coordinates to the format used by the undistortPoints function
        pts_cam_1 = np.array([coords_cam_1], dtype=np.float32).reshape(2, 1)
        pts_cam_2 = np.array([coords_cam_2], dtype=np.float32).reshape(2, 1)

        # Undistort points
        undistorted_pts_cam_1 = cv2.undistortPoints(pts_cam_1, K_1, d_1).ravel()
        undistorted_pts_cam_2 = cv2.undistortPoints(pts_cam_2, K_2, d_2).ravel()

        # Triangulate points (coords in chessboard frame)
        points_4D_chess = cv2.triangulatePoints(P_1, P_2, undistorted_pts_cam_1, undistorted_pts_cam_2)

        # Convert to 3D coordinates (in cm )
        points_3D_chess = (points_4D_chess[:3] / points_4D_chess[3]).ravel()

        # # Tests
        # # Test get rectification matrix
        # T_21 = self.cam_2_projection_mat
        # R_21 = T_21[:3,:3]
        # t_21 = T_21[:2,3]
        # Rectif_1, Rectif_2, P_1, P_2 = cv2.stereoRectify(K1, d1, K2, d2, self.img_size, R_21, t_21)
        # t_21 = np.array([-263.13261, 0.0, 0.0]).reshape(3,1)

        # cam_1_proj_mat = np.matmul(K_1,np.eye(3,4))
        # # print(cam_1_proj_mat)

        # R_2T_R_1 = np.matmul(np.transpose(R_2),R_1)
        # R_2T_t_2 = np.matmul(np.transpose(R_2),t_2)
        # rectif = np.concatenate((R_2T_R_1, R_2T_t_2), axis=1)
        # cam_2_proj_mat = np.matmul(K_2,rectif)
        # # print(cam_2_proj_mat)
        # points_4D_test = cv2.triangulatePoints(cam_1_proj_mat, cam_2_proj_mat, undistorted_pts_cam_1, undistorted_pts_cam_2)

        # # Convert to 3D coordinates
        # # triangulatePoints() outputs the coords in the rectified coord system associated to cam_1
        # points_3D_test = (points_4D_test[:3] / points_4D_test[3]).ravel()

        # Transformation from robot world frame to chessboard frame
        T_chess_W = np.eye(4)
        R_chess_W = R.from_euler("zyx", [180, -90, 0], degrees=True) .as_matrix()
        t_chess_W = np.array([-15, 15, 2])  # Origin of the chessboard frame according to CAD
        T_chess_W[:3, :3] = R_chess_W
        T_chess_W[:3, 3] = t_chess_W

        points_4D_world = np.matmul(np.linalg.inv(T_chess_W), points_4D_chess.reshape(4, 1))
        points_3D_world = (points_4D_world[:3]/points_4D_world[3]).ravel()

        # Add offset (TODO: Measure offset in CAD model)
        offset = np.array([-16, -4.01, 16.73])  # in cm
        # offset = np.array([0.0, 0.0, 0.0])
        points_3D_world = points_3D_world + offset

        return points_3D_chess, points_3D_world

    def save_recorded_pts(self):
        # Define the CSV file name and save directory
        dirname = os.path.dirname(__file__)
        save_dir = os.path.join(dirname, '../config')
        csv_file = os.path.join(save_dir, 'recorded_pts.csv')

        # Open the CSV file in write mode
        with open(csv_file, mode='w', newline='') as file:
            writer = csv.writer(file)

            # Write a header row (optional)
            writer.writerow(['id', 'x', 'y', 'z'])

            # Write the 3D points to the CSV file with ids
            for idx, point in enumerate(self.recorded_pts, start=1):
                writer.writerow([idx, point[0], point[1], point[2]])

        print(f"3D points have been written to {csv_file}")
        return

    def node_handle(self, points_3D):
        # ==================================================
        # Publish images
        # ==================================================
        # msg_left = self.bridge.cv2_to_imgmsg(frame_cam_1, 'bgr8')
        # msg_right = self.bridge.cv2_to_imgmsg(frame_cam_2, 'bgr8')
        # self.img_1_publisher.publish(msg_left)
        # self.img_2_publisher.publish(msg_right)

        # ==================================================
        # Publish the needle marker transform message
        # ==================================================
        scale_factor = 0.01  # To transform cm in m for use in rviz
        now = rclpy.time.Time()

        # Read needle orientation according to model
        if self.tf_buffer.can_transform('panto_link_fulcrum', 'tool_phi_link', now):
            tf_PI_tool_phi = self.tf_buffer.lookup_transform('panto_link_fulcrum', 'tool_phi_link', now)

            rotation = np.array([tf_PI_tool_phi.transform.rotation.x,
                                 tf_PI_tool_phi.transform.rotation.y,
                                 tf_PI_tool_phi.transform.rotation.z,
                                 tf_PI_tool_phi.transform.rotation.w,])

        else:
            self.get_logger().info("Could not transform from 'panto_link_fulcrum' to 'tool_phi_link'")
            rotation = np.array([0.0, 0.0, 0.0, 0.0])

        # Update needle_tip_marker frame
        tf_marker_msg = TransformStamped()
        tf_marker_msg.header.stamp = self.get_clock().now().to_msg()
        tf_marker_msg.header.frame_id = 'world'
        tf_marker_msg.child_frame_id = 'needle_tip_marker'
        tf_marker_msg.transform.translation.x = float(scale_factor * points_3D[0])
        tf_marker_msg.transform.translation.y = float(scale_factor * points_3D[1])
        tf_marker_msg.transform.translation.z = float(scale_factor * points_3D[2])
        tf_marker_msg.transform.rotation.x = rotation[0]
        tf_marker_msg.transform.rotation.y = rotation[1]
        tf_marker_msg.transform.rotation.z = rotation[2]
        tf_marker_msg.transform.rotation.w = rotation[3]
        self.tf_broadcaster.sendTransform(tf_marker_msg)

        # ==================================================
        # Test update needle interaction link position
        # ==================================================
        needle_lenght = 20.0  # (in cm)

        if self.tf_buffer.can_transform('panto_link_fulcrum', 'needle_tip_marker', now):
            tf_marker_PI = self.tf_buffer.lookup_transform('panto_link_fulcrum', 'needle_tip_marker', now)

            needle_vect = np.array([tf_marker_PI.transform.translation.x,
                                    tf_marker_PI.transform.translation.y,
                                    tf_marker_PI.transform.translation.z])

            # Compute the distance between the marker and the pantograph end effector
            insertion_length = np.linalg.norm(needle_vect)

        else:
            self.get_logger().info("Could not transform from 'panto_link_fulcrum' to 'needle_tip_marker'")
            insertion_length = 0.0

        # Update P_U frame
        P_U_link_msg = TransformStamped()
        P_U_link_msg.header.stamp = self.get_clock().now().to_msg()
        P_U_link_msg.header.frame_id = 'tool_phi_link'
        P_U_link_msg.child_frame_id = 'needle_interaction_link'
        P_U_link_msg.transform.translation.x = 0.0
        P_U_link_msg.transform.translation.y = 0.0
        P_U_link_msg.transform.translation.z = (scale_factor * needle_lenght) - insertion_length
        P_U_link_msg.transform.rotation.x = 0.0
        P_U_link_msg.transform.rotation.y = 0.0
        P_U_link_msg.transform.rotation.z = 0.0
        P_U_link_msg.transform.rotation.w = 1.0

        # Broadcast the new transform
        self.tf_broadcaster.sendTransform(P_U_link_msg)

    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        return np.array([x, y, z, w])

    def nothing(self, x):
        # Function callBack for trackbars
        # Dos nothing
        pass


def main(args=None):
    rclpy.init(args=args)
    node = StereoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()
    node.cap_1.release()
    node.cap_2.release()


if __name__ == '__main__':
    main()
