# Robot Raconteur Camera Driver

Simple camera driver using OpenCV backend to capture images. Implements `com.robotraconteur.imaging.Camera` standard type.

Example usage:

    python robotraconteur_camera_driver.py --width=1280 --height=720 --fps=20 --device-id=0

On Ubuntu, use `python3` instead of `python`

All standard Robot Raconteur command line options are also supported. See https://github.com/robotraconteur/robotraconteur/wiki/Command-Line-Options . Use `--robotraconteur-tcp-port=` and `--robotraconteur-nodname=` to change the TCP port and node name.

The Robot Raconteur Python and OpenCV modules are required. On Windows and Mac OSX install using:

    python -m pip install robotraconteur opencv-contrib-python

On Ubuntu, the Robot Raconteur PPA must be installed. See https://github.com/robotraconteur/robotraconteur/blob/master/README.md#ubuntu-xenial-bionic-and-focal

After the PPA is installed:

    sudo apt-get install python3-robotraconteur python3-opencv

License: Apache 2.0
