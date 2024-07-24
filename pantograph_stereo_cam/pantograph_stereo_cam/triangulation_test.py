import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
import os

# Initialize global variables to store coordinates
coords_img1 = None
coords_img2 = None


def main():
    global coords_img1, coords_img2
    # Read stereo system params from yaml files
    dirname = os.path.dirname(__file__)
    # Camera 1
    cam_1_params_path = os.path.join(dirname, '../config/cam_0_info.yaml')
    # Camera 2
    cam_2_params_path = os.path.join(dirname, '../config/cam_1_info.yaml')

    # Read the images
    img1_path = os.path.join(dirname, '../config/frames_pair/camera0_2.png')
    img2_path = os.path.join(dirname, '../config/frames_pair/camera1_2.png')
    img1 = cv2.imread(img1_path)
    img2 = cv2.imread(img2_path)

    # Ensure the images are the same height for horizontal concatenation
    height1, width1 = img1.shape[:2]
    height2, width2 = img2.shape[:2]

    if height1 != height2:
        # Resize the second image to have the same height as the first image
        img2 = cv2.resize(img2, (width2, height1))

    # Concatenate the images horizontally
    combined_img = np.hstack((img1, img2))

    # Create a named window
    cv2.namedWindow('Triangulation test')

    # Set the mouse callback function
    cv2.setMouseCallback('Triangulation test', mouse_callback, {'img1_width': width1})

    # Display the combined image
    while True:
        display_img = combined_img.copy()

        # Add legend to the images
        add_legend(display_img, coords_img1, (10, height1))
        add_legend(display_img, coords_img2, (width1 + 10, height1))
        cv2.imshow('Triangulation test', display_img)

        # Break the loop when 'q' is pressed
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

        elif key == ord('t'):
            triangulation(coords_img1, coords_img2, cam_1_params_path, cam_2_params_path)

    # Destroy all windows
    cv2.destroyAllWindows()


# Define the mouse callback function
def mouse_callback(event, x, y, flags, param):
    global coords_img1, coords_img2
    if event == cv2.EVENT_LBUTTONDOWN:
        img1_width = param['img1_width']
        if x < img1_width:
            print(f"Mouse clicked at ({x}, {y}) in Image 1")
            coords_img1 = (x, y)
        else:
            print(f"Mouse clicked at ({x - img1_width}, {y}) in Image 2")
            coords_img2 = (x - img1_width, y)


# Function to add coordinates legend to an image
def add_legend(image, coords, position):
    if coords is not None:
        text = f"Mouse coords : ({coords[0]}, {coords[1]})"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        color = (255, 0, 0)
        thickness = 2

        text_x = position[0]
        text_y = position[1] - 10
        cv2.putText(image, text, (text_x, text_y), font, font_scale, color, thickness, cv2.LINE_AA)


def triangulation(coords1, coords2, cam_1_params_path, cam_2_params_path):
    # Extract camera parameters from yaml files
    with open(cam_1_params_path) as f1:
        data_1 = yaml.safe_load(f1)

    K_1 = np.array(data_1['camera_matrix']['data']).reshape(3, 3)
    d_1 = np.array(data_1['distortion_coefficients']['data']).reshape(-1, 5)
    P_1 = np.array(data_1['projection_matrix']['data']).reshape(3, 4)
    # R_1 = np.array(data_1['rectification_matrix']['data']).reshape(3, 3)

    with open(cam_2_params_path) as f2:
        data_2 = yaml.safe_load(f2)

    K_2 = np.array(data_2['camera_matrix']['data']).reshape(3, 3)
    d_2 = np.array(data_2['distortion_coefficients']['data']).reshape(-1, 5)
    P_2 = np.array(data_2['projection_matrix']['data']).reshape(3, 4)
    # R_2 = np.array(data_2['rectification_matrix']['data']).reshape(3, 3)

    if coords1 and coords2:
        # ================================================================================================
        # Triangulation
        # ================================================================================================
        print(f"Triangulating points: Image 1 -> {coords1}, Image 2 -> {coords2}")
        # Undistort points
        undistorted_pts_cam_1 = cv2.undistortPoints(coords_img1, K_1, d_1)
        undistorted_pts_cam_2 = cv2.undistortPoints(coords_img2, K_2, d_2)
        point_4D = cv2.triangulatePoints(P_1, P_2, undistorted_pts_cam_1, undistorted_pts_cam_2)
        # Normalize point
        point_3D = (point_4D[:3] / point_4D[3]).ravel()

        print('3D coordinates calculated by first method : \n', point_3D)

        # # Test
        # t_2 = np.array([-263.13261, 0.0, 0.0]).reshape(3,1)

        # cam_1_proj_mat = np.matmul(K_1,np.eye(3,4))

        # R_2T_R_1 = np.matmul(np.transpose(R_2),R_1)
        # R_2T_t_2 = np.matmul(np.transpose(R_2),t_2)
        # rectif = np.concatenate((R_2T_R_1, R_2T_t_2), axis=1)
        # cam_2_proj_mat = np.matmul(K_2,rectif)

        # points_4D_test = cv2.triangulatePoints(cam_1_proj_mat, cam_2_proj_mat, undistorted_pts_cam_1, undistorted_pts_cam_2)

        # # Convert to 3D coordinates
        # # triangulatePoints() outputs the coords in the rectified coord system associated to cam_1
        # points_3D_test = (points_4D_test[:3] / points_4D_test[3]).ravel()

        # print('3D coordinates calculated by second method : \n', points_3D_test)
    else:
        print("Both coordinates must be set to perform triangulation")
    return


if __name__ == "__main__":
    main()
