# Robo3-2020-7_final

### Operation check

https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

```
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

sudo apt update

sudo apt install librealsense2-dkms

sudo apt install librealsense2-utils

sudo apt install librealsense2-dev

sudo apt install librealsense2-dbg

realsense-viewer
``` 

### build


https://github.com/IntelRealSense/realsense-ros

```

export ROS_VER=noetic

sudo apt-get install ros-$ROS_VER-realsense2-camera

cd ~/catkin_ws/src/

git clone https://github.com/IntelRealSense/realsense-ros.git

cd realsense-ros/

git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

cd ..

catkin_init_workspace

cd ..

catkin_make clean

catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release

catkin_make install

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc

roslaunch realsense2_camera rs_camera.launch

```
