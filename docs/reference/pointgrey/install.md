# PointGrey Install

Install Pointgrey camera drivers

    sudo apt-get install ros-indigo-pointgrey-camera-driver

Add a udev rule (one line)

    SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="2002",
    GROUP="plugdev", SYMLINK+="firefly", MODE:="0666"

Restart udev

    sudo service udev restart

Check to see if camera is detected

    rosrun pointgrey_camera_driver list_cameras

Run the camera with

    roslaunch pointgrey_camera_driver camera.launch

