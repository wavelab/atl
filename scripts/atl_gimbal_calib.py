#!/usr/bin/env python
from time import sleep
from os.path import join

import cv2
import rospy
import numpy as np

from atl import Gimbal


class Chessboard(object):
    def __init__(self, rows, cols, sq_size):
        self.rows = rows
        self.cols = cols
        self.sq_size = sq_size
        self.cb_size = (self.cols, self.rows)


def move_gimbal():
    rospy.init_node("atl_gimbal_calib")
    gimbal = Gimbal()
    rospy.sleep(1.0)

    gimbal.set_attitude([0.0, 0.0, 0])
    sleep(1)

    roll = 0.0
    for i in range(5):
        print("ROLL: " + str(roll))

        pitch = 0.0
        for j in range(10):
            print("PITCH: " + str(pitch))
            gimbal.set_attitude([roll, pitch, 0])
            sleep(2)
            pitch += 0.1

        roll += 0.1
        print("")


def find_chessboard(img, cb_size, camera_id):
    # find chessboard
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH
    flags += cv2.CALIB_CB_NORMALIZE_IMAGE
    flags += cv2.CALIB_CB_FAST_CHECK
    ret, img_points = cv2.findChessboardCorners(gray, cb_size, None, flags)

    # draw chessboard
    if ret is True:
        # refine image points with corner sub-pixel correction
        # criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        #             30,
        #             0.001)
        # img_points = cv2.cornerSubPix(gray,
        #                             img_points,
        #                             (11, 11),
        #                             (-1, -1),
        #                             criteria)

        img_chessboard = cv2.drawChessboardCorners(img,
                                                   cb_size,
                                                   img_points,
                                                   ret)
        cv2.imshow("Chessboard " + str(camera_id), img_chessboard)
        return img_points

    else:
        print("Failed to detected chessboard in image!")
        return None


def output_jason_format(output_path,
                        object_points,
                        img_points,
                        camera_matrix,
                        dist_coef,
                        encoder_data):
    # calculate transformation matrix
    retval, rvec, tvec = cv2.solvePnP(object_points,
                                      img_points,
                                      camera_matrix,
                                      dist_coef,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
    if retval is False:
        raise RuntimeError("solvePnP failed!")

    # convert rotation vector to matrix
    R = np.zeros((3, 3))
    cv2.Rodrigues(rvec, R)

    # form transformation matrix
    tmatrix = np.array([[R[0][0], R[0][1], R[0][2], tvec[0]],
                        [R[1][0], R[1][1], R[1][2], tvec[1]],
                        [R[2][0], R[2][1], R[2][2], tvec[2]],
                        [0.0, 0.0, 0.0, 1.0]])

    # open file for output
    jason_file = open(output_path, "w+")

    # gridpoints
    jason_file.write("gridpoints:\n")
    for i in range(len(object_points)):
        obj_pt = object_points[i]
        img_pt = img_points[i][0]
        jason_file.write(" ".join(str(x) for x in obj_pt))
        jason_file.write(" ")
        jason_file.write(" ".join(str(x) for x in img_pt))
        jason_file.write("\n")

    # tmatrix
    jason_file.write("tmatrix:\n")
    for i in range(4):
        for j in range(4):
            jason_file.write(str(tmatrix[i][j]))
            jason_file.write(" ")
        jason_file.write("\n")

    # gimbal angles
    jason_file.write("gimbalangles:\n")
    jason_file.write(" ".join(str(x) for x in encoder_data[:2]))
    jason_file.write("\n")

    # end
    jason_file.write("end:\n")


def convert_measurements_to_jason_format(path,
                                         chessboard,
                                         static_camera_K,
                                         static_camera_d,
                                         gimbal_camera_K,
                                         gimbal_camera_d):
    # hard-coding the object points - assuming chessboard is origin by
    # setting chessboard in the x-y plane (where z = 0).
    object_points = []
    for i in range(chessboard.rows):
        for j in range(chessboard.cols):
            pt = [j * chessboard.sq_size, i * chessboard.sq_size, 0.0]
            object_points.append(pt)
    object_points = np.array(object_points)

    # number of measurements
    encoder_file = open(join(path, "gimbal_encoder.dat"))
    encoder_lines = encoder_file.readlines()
    encoder_data = []
    for line in encoder_lines:
        measurement = [float(x) for x in line.split(",")]
        encoder_data.append(measurement)
    nb_measurements = len(encoder_data)

    # loop through camera images
    for i in range(nb_measurements):
        img1 = cv2.imread(join(path, "static_camera", "img_%d.jpg" % i))
        img2 = cv2.imread(join(path, "gimbal_camera", "img_%d.jpg" % i))

        # find and draw chessboard
        img_points1 = find_chessboard(img1, chessboard.cb_size, 1)
        img_points2 = find_chessboard(img2, chessboard.cb_size, 2)

        # output jason format
        output_jason_format(join(path, "%d_static.txt" % i),
                            object_points,
                            img_points1,
                            static_camera_K,
                            static_camera_d,
                            encoder_data[i])

        output_jason_format(join(path, "%d_gimbal.txt" % i),
                            object_points,
                            img_points2,
                            gimbal_camera_K,
                            gimbal_camera_d,
                            encoder_data[i])

        # parse keyboard input
        key = cv2.waitKey(0)
        if key == 113 or key == 27:
            exit(0)


if __name__ == "__main__":
    # rospy.init_node("atl_gimbal_calib")
    # gimbal = Gimbal()
    # rospy.sleep(1.0)
    #
    # gimbal.activate(False)

    # chessboard settings
    chessboard = Chessboard(8,  # rows
                            8,  # cols
                            0.083)  # square size

    # static camera
    static_camera_K = np.array([[368.083519, 0.000000, 323.059860],
                               [0.000000, 368.380343, 254.482050],
                               [0.000000, 0.000000, 1.000000]])

    static_camera_d = np.array([-0.317398, 0.084401, -0.000403, 0.001252])

    # gimbal camera
    gimbal_camera_K = np.array([[386.468175, 0.000000, 326.873225],
                               [0.000000, 386.641069, 235.500697],
                               [0.000000, 0.000000, 1.000000]])
    gimbal_camera_d = np.array([-0.299881, 0.073809, -0.001179, 0.001184])

    # convert collected data
    convert_measurements_to_jason_format("/tmp/calibration",
                                         chessboard,
                                         static_camera_K,
                                         static_camera_d,
                                         gimbal_camera_K,
                                         gimbal_camera_d)
