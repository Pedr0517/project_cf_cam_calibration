import numpy as np
import cv2
import PIL
import os
from cv2 import aruco
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import matplotlib as mpl
from argparse import ArgumentParser


def read_chessboards(images, minimum_detected_markers, aruco_dict, board):
    """
    Charuco base pose estimation.
    """
    print("POSE ESTIMATION STARTS:")
    allCorners = []
    allIds = []
    decimator = 0
    # SUB PIXEL CORNER DETECTION CRITERION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

    for im in images:
        print("=> Processing image {0}".format(im))
        frame = cv2.imread(im)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict)
        if len(ids) < minimum_detected_markers:
            print("Not enough markers found in image {0}".format(im))
            continue

        if len(corners) > 0:
            # SUB PIXEL DETECTION
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(3, 3),
                                 zeroZone=(-1, -1),
                                 criteria=criteria)
            res2 = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if res2[1] is not None and res2[2] is not None and len(res2[1]) > 3 and decimator % 1 == 0:
                allCorners.append(res2[1])
                allIds.append(res2[2])

        decimator += 1

    imsize = gray.shape
    return allCorners, allIds, imsize


def calibrate_camera(allCorners, allIds, imsize, board):
    """
    Calibrates the camera using the dected corners.
    """
    print("CAMERA CALIBRATION")

    cameraMatrixInit = np.array([[1000., 0., imsize[0] / 2.],
                                 [0., 1000., imsize[1] / 2.],
                                 [0., 0., 1.]])

    distCoeffsInit = np.zeros((5, 1))
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    # flags = (cv2.CALIB_USE_INTRINSIC_GUESS + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO)
    # flags = (cv2.CALIB_RATIONAL_MODEL)
    (ret, camera_matrix, distortion_coefficients0,
     rotation_vectors, translation_vectors,
     stdDeviationsIntrinsics, stdDeviationsExtrinsics,
     perViewErrors) = cv2.aruco.calibrateCameraCharucoExtended(
        charucoCorners=allCorners,
        charucoIds=allIds,
        board=board,
        imageSize=imsize,
        cameraMatrix=cameraMatrixInit,
        distCoeffs=distCoeffsInit,
        flags=flags,
        criteria=(cv2.TERM_CRITERIA_EPS & cv2.TERM_CRITERIA_COUNT, 10000, 1e-9))

    return ret, camera_matrix, distortion_coefficients0, rotation_vectors, translation_vectors


def plot_aruco_board(aruco_board):
    """
    Plot the charuco board.
    """
    imboard = board.draw((2000, 2000))
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    plt.imshow(imboard, cmap=mpl.cm.gray, interpolation="nearest")
    ax.axis("off")
    plt.show()


def show_undistorted(frame, mtx, dist):
    """
    Show the undistorted image.
    """
    img_undist = cv2.undistort(frame, mtx, dist, None)
    plt.subplot(1, 2, 1)
    plt.imshow(frame)
    plt.title("Raw image")
    plt.axis("off")
    plt.subplot(1, 2, 2)
    plt.imshow(img_undist)
    plt.title("Corrected image")
    plt.axis("off")
    plt.show()


def print_calib_matrix(mtx, dist, all_distortion_coefficients=False):
    # print("Camera calibration matrix: ")
    # print matrix without scientific notation
    np.set_printoptions(suppress=True)

    # print("Camera distortion coefficients: ")
    if all_distortion_coefficients:
        dist_val = dist.flatten()
    else:
        dist_val = dist.flatten()[:7]

    return mtx, dist_val


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

    print_calib_matrix(mtx, dist, args.all_distortion_coefficients)

    if args.show_undistorted:
        for i in range(0, len(images)):
            frame = cv2.imread(images[i])
            show_undistorted(frame, mtx, dist)
