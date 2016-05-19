#!/bin/bash
set -e  # halt on first error

bash ros_install.bash
bash apriltag_install.bash
bash pointgrey_install.bash
bash navio2_cpp_install.bash
