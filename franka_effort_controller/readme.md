============================
#one shell start
sudo su
cd /home/robot/robot/src
./start.sh

============================
#init system, to set cpu dma latency, or 
sudo su
#passwd 123456
cd /home/robot/robot/devel/lib/device_driver
./set_cpu_dma_latency

===============================================
#ethercat config
sudo su
#passwd 123456
cd /home/robot/robot/src/EtherCATMaster
./ECMenv_rc.sh
./runECM.sh
export LD_LIBRARY_PATH=/home/robot/robot/src/control_lib/lib:$LD_LIBRARY_PATH

============================
#load arm controller 
sudo su
roslaunch device_driver device_driver.launch
============================
#start controller
roslaunch device_driver start_controller.launch controller_name:=joint_trajectory_controller
============================
#ros moveit
roslaunch th_moveit_config moveit_real_robot.launch
============================
#stop controller
roslaunch device_driver stop_controller.launch controller_name:=joint_trajectory_controller
============================
#start admittance control
roslaunch robotiq_ft_sensor start_admittance.launch 
#stop admittance control
roslaunch robotiq_ft_sensor stop_admittance.launch 
============================
#kill ethercat master
sudo pkill y2
============================
============================
#Moveit  backup
(1)moveit 和 gazebo
roslaunch th_cobot_moveit_config demo_gazebo.launch
(2)moveit (单独）
roslaunch th_cobot_moveit_config th_demo.launch
(3)moveit (仿真）
只动moveit
roslaunch th_moveit_config demo.launch
(3)moveit 和 实物
roslaunch th_moveit_config moveit_real_robot.launch
(4)编程
 rosrun th_demo motion_demo.py
============================
sudo su   
cd /robot
./HYYRobotMain

./RobotMain --path /home/robot/robot/src/robot_config

=============================
Arduino Pressure Control:
rosrun rosserial_python serial_node.py _port:=/dev/arduino_mega _baud:=57600
rosrun th_demo simple_kv_test.py