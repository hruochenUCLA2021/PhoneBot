source install/setup.bash
(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge$ 
ros2 launch robot_visualizer phonebot_visualizer.launch.py
(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge$ 
ros2 run phonebot_bridge phonebot_udp_bridge_immediate --ros-args -p android_ip:=192.168.20.31
ros2 run phonebot_bridge phonebot_udp_bridge_immediate --ros-args -p android_ip:=192.168.20.21
(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge$ 
ros2 run phonebot_bridge phonebot_example_motor_pub --ros-args -p hz:=2.0



order on raspberry pi zero 2w:
pkill -f cursor-server

echo 'alias activate_phonebot="source /opt/ros/humble/setup.bash && source ~/PhoneBot_3_10/bin/activate && export PYTHONPATH=\$PYTHONPATH:~/PhoneBot_3_10/lib/python3.10/site-packages"' >> ~/.bashrc

activate_phonebot
