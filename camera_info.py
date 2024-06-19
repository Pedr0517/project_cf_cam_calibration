import os
from cv2 import aruco
import cv2
import numpy as np
from argparse import ArgumentParser
from charucoCalib import plot_aruco_board, read_chessboards, calibrate_camera, print_calib_matrix, show_undistorted


def cam_info(mtx, distorted):
    K_mtx = mtx

    distorted_val = distorted[:5]

    R_mtx = np.array([[1, 0, 0],
                      [0, 1, 0],
                      [0, 0, 1]])

    P_mtx = np.array([[K_mtx[0][0], 0, K_mtx[0][2], 0],
                      [0, K_mtx[1][1], K_mtx[1][2], 0],
                      [0, 0, 1, 0]])

    print("D: ")
    print(distorted_val)
    print()
    print("K: ")
    print(K_mtx)
    print()
    print("R: ")
    print(R_mtx)
    print()
    print("P: ")
    print(P_mtx)


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
    board = cv2.aruco.CharucoBoard((11, 11), .1, .08, aruco_dict)

    if args.show_aruco_board:
        plot_aruco_board(board)

    datadir = args.datadir
    images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".png")])

    allCorners, allIds, imsize = read_chessboards(
        images, args.minimun_detected_markers, aruco_dict, board)
    ret, mtx, dist, rvecs, tvecs = calibrate_camera(allCorners, allIds, imsize, board)

    mtx, distorted = print_calib_matrix(mtx, dist, args.all_distortion_coefficients)
    cam_info(mtx, distorted)
