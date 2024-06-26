from charucoCalib import plot_aruco_board, read_chessboards, calibrate_camera, print_calib_matrix, show_undistorted
from argparse import ArgumentParser
import numpy as np
import cv2
from cv2 import aruco
import os
import yaml


def cam_info(mtx, distorted, size):
    """Creating text file with calibration info formatted"""

    # note, change path when seperating calibrations
    file_path = "camera_calibration_info.yml"
    open(file_path, 'w').close()

    K_mtx = mtx.flatten()
    print(K_mtx)

    distorted_val = distorted[:5]

    R_mtx = np.array([1, 0, 0,
                     0, 1, 0,
                     0, 0, 1])

    P_mtx = np.array([K_mtx[0], 0, K_mtx[2], 0,
                      0, K_mtx[4], K_mtx[5], 0,
                      0, 0, 1, 0])

    # Organizing calibration data#
    print("D: ")
    print(distorted_val)
    # add_file(file_path, distorted_val)
    print()
    print("K: ")
    print(K_mtx)
    # add_file(file_path, K_mtx)
    print()
    print("R: ")
    print(R_mtx)
    # add_file(file_path, R_mtx)
    print()
    print("P: ")
    print(P_mtx)
    # add_file(file_path, 'P', P_mtx)

    # Extra info#
    img_width = size[1]
    img_height = size[0]
    camera = "camera"
    dist_model = """plumb_bob"""

    # Dictionary for YAML file#
    cam_data = {
        "//**": {
            "ros_parameters": {
                "camera": {
                    "img_width": img_width,
                    "img_height": img_height,
                    "camera_name": camera,
                    "camera_matrix": K_mtx.tolist(),
                    "distortion_model": dist_model,
                    "distortion_coefficients": distorted_val.tolist(),
                    "rectification_matrix": R_mtx.tolist(),
                    "projection_matrix": P_mtx.tolist()
                }
            }
        }
    }

    with open(file_path, 'w') as f:
        yaml.dump(cam_data, f, default_flow_style=None, sort_keys=False)


if __name__ == "__main__":

    parser = ArgumentParser(description="Calibrate camera using charuco board.")
    parser.add_argument("--datadir", type=str, default="./drone_images_auto/",
                        help="Directory containing images.", required=True)
    parser.add_argument("--show-aruco-board", action="store_true", help="Show the aruco board.")
    parser.add_argument("--show-undistorted", action="store_true",
                        help="Show the undistorted images.")
    parser.add_argument("--minimun-detected-markers", type=int, default=6,
                        help="Minimum number of markers to detect in the image.")
    parser.add_argument("--all_distortion_coefficients", action="store_true",
                        help="Show all distortion coefficients.")

    args = parser.parse_args()

    # aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    # board = aruco.CharucoBoard_create(11, 11, .1, .08, aruco_dict)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    board = cv2.aruco.CharucoBoard((11, 11), .019, .015, aruco_dict)

    if args.show_aruco_board:
        plot_aruco_board(board)

    datadir = args.datadir
    images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png")])

    allCorners, allIds, imsize = read_chessboards(
        images, args.minimun_detected_markers, aruco_dict, board)

    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners, allIds, imsize, board)

    mtx, distorted = print_calib_matrix(mtx, dist, args.all_distortion_coefficients)
    cam_info(mtx, distorted, imsize)

    if args.show_undistorted:
        for i in range(0, len(images)):
            frame = cv2.imread(images[i])
            show_undistorted(frame, mtx, dist)
