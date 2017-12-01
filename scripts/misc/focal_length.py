#!/usr/bin/env python3
from math import pi
from math import tan


def deg2rad(d):
    return d * pi / 180.0


def focal_length(image_width, image_height, fov):
    fx = (image_width / 2.0) / tan(deg2rad(fov) / 2.0)
    fy = (image_height / 2.0) / tan(deg2rad(fov) / 2.0)
    return (fx, fy)

if __name__ == "__main__":
    fx, fy = focal_length(640, 480, 110)
    print(fx)
    print(fy)
