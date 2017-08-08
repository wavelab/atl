#!/usr/bin/env python
from time import sleep

import rospy

from atl import Gimbal


if __name__ == "__main__":
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
