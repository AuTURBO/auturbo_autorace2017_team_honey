# autorace2017-team-honey
AutoRace 2017 in RoboWorld 2017 RBiz Challenge, Seoul Korea / Team Honey

This package refers to the [MIT Duckie Town source](https://github.com/duckietown/Software)

First you install turtlebot3 default setting. 
http://turtlebot3.robotis.com/en/latest/

# Install
The following are needed to be installed or downloaded

* turtlebot3_RBIZ package

  ```bash
  $ ~/catkin_ws/src && git clone https://github.com/hyunoklee/turtlebot3_RBIZ.git
  ```
* install the others

  ```bash
  $ sudo apt-get install libv4l-dev
  ```

  ```bash
  $ sudo apt-get install ros-kinetic-tf-conversions ros-kinetic-cv-bridge ros-kinetic-image-transport ros-kinetic-camera-info-manager ros-kinetic-theora-image-transport ros-kinetic-joy ros-kinetic-image-proc -y
  ```

  ```bash
  $ sudo apt-get install ros-kinetic-compressed-image-transport -y
  ```

  ```bash
  $ sudo apt-get install libyaml-cpp-dev -y
  ```

  ```bash
  $ sudo apt-get install libblas-dev liblapack-dev libatlas-base-dev gfortran
  ```

# Run
These packages can be launched by following commands

* run roscore node ( remote pc side )

  ```bash
  $ roscore
  ```

* run camera , openCR, Lidar node ( turtle_bot side )

  ```bash
  $ roslaunch turtlebot3_auto_sensor image_get_usb_camera.launch
  $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
  ```
* run line detection node  ( remote pc side ) 

  ```bash
  $ roslaunch line_detect detect_line.launch
  ```

  ```bash
  $ Check the operation up to here 
  ```

* run line filter node  ( remote pc side ) 

  ```bash
  $ roslaunch lane_filter lane_filter_node.launch
  ```

* run motor control node  ( remote pc side ) 

  ```bash
  $ roslaunch motor_control motor_controller_node.launch

# 다운로드 받고 실행 할때 아래 파일 경로를 자신의 폴더 위치에 맞게 변경하세요 
  -> turtlebot3_auto/line_detect/src/detect_line.py  파일의 아래 부분 
      with open("/home/rt/catkin_ws/src/turtlebot3_auto/line_detect/src/callibra_ros.yaml")
     어차피  기체가 바뀌면 cal 값  그대로 사용하지 못합니다.  
# 광원이 변경이 되면 해당 위치에서 라인에 맞는  HSV 값을 변경해 필요,( 주요 TUNNING POINT )
  -> turtlebot3_auto/line_detect/config/universal_HSV.yaml
      hsv_yellow1: [10,70,100] #[25,70,100]#[25,100,120]#[25,50,90] 
      hsv_yellow2: [55,255,255] #[45,255,255]

# 현재 파킹 막아 놨습니다. 라인, 터널 만 됩니다.
  터널모드 는 그냥 장애물 피하기만 함.

# Tool

* view image

  ```bash
  $ rviz
  $ add  ->  By topic -> 
  ```

* view topic

  ```bash
  $ rqt
  $ 
  ```

# Camera Calibration

* Calibration install package

  ```bash
  $ rosdep install camera_calibration
  ```

* Calibration package run
  ```bash
  $ roslaunch turtlebot3_auto_sensor image_get_usb_camera.launch
  ```
  ```bash
  $ rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.031 image:=/image_raw camera:=/
  ```

  ```bash
  $ use calibration pattern calibration pattern.pdf 
  and save calibration data  -> turtlebot3_auto/line_detect/src/callibra_ros.yaml
  ```
  

