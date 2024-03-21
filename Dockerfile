ARG ROS_DISTRO=galactic

######
#  Base Image for Ros2 
######
#FROM nvcr.io/nvidia/l4t-base:r36.2.0
FROM nvcr.io/nvidia/l4t-cuda:11.4.19-devel

SHELL ["/bin/bash", "-c"]

## psure the two following lines are unnecessary unless this image comes loaded with opencv
#RUN apt purge  libopencv* python-opencv
#RUN find /usr/local/ -name "*opencv*" -exec rm -i {} \;

ENV DEBIAN_FRONTEND=noninteractive
RUN apt update && apt -y dist-upgrade

## build OpenCV 
COPY install_opencv4.4.0_Jetson.sh .
RUN ./install_opencv4.4.0_Jetson.sh 

RUN apt install -y software-properties-common && apt-add-repository universe

#RUN apt update && apt -y install curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt install -y --fix-missing --no-install-recommends ros-galactic-desktop
#ADD "https://www.random.org/cgi-bin/randbyte?nbytes=10&format=h" skipcache
RUN apt install -y python3-colcon-common-extensions

##  Install vision_opencv, cv_bridge
RUN apt install libboost-python-dev
RUN mkdir -p ros2_ws/src
WORKDIR ros2_ws
RUN cd src && git clone https://github.com/theguyinthe-box/vision_opencv.git -b galactic
RUN source /opt/ros/galactic/setup.bash && colcon build --symlink-install && . install/setup.bash 
RUN cd src && mv vision_opencv /

## ros2_msg
RUN cd src && git clone https://github.com/AveesLab/ros2_msg.git
RUN source install/setup.bash && colcon build --symlink-install && . install/setup.bash

## object_detection lidar
RUN apt install -y ros-galactic-perception-pcl
RUN cd src && git clone https://github.com/theguyinthe-box/object_detection_ros2.git

## Lane Detection
RUN cd src && git clone https://github.com/AveesLab/lane_detection_ros2.git

## yolo_object_detection
RUN cd src && git clone https://github.com/AveesLab/yolo_object_detection_ros2.git
RUN cd src/yolo_object_detection_ros2/darknet && make -j12

#COPY id_rsa .
#RUN git config --add --global core.sshCommand 'ssh -i /ros2_ws/id_rsa'
#RUN cd src && git clone git@github.com:theguyinthe-box/scale_truck_control_ros2_carla.git
RUN apt install -y ros-galactic-ackermann-msgs
COPY scale_truck_control_ros2_carla src/
RUN source install/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug && . install/setup.bash

CMD source install/setup.bash && ros2 launch scale_truck_control_ros2 LV_carla.launch.py