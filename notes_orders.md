ssh hrc@192.168.20.33
using apollo network 
romela111111


HWdriver 



radxa board:
rock
rock

192.168.20.34
also apollo

ssh rock@192.168.20.34



source install/setup.bash
(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge$ 
ros2 launch robot_visualizer phonebot_visualizer.launch.py
(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge$ 
ros2 run phonebot_bridge phonebot_udp_bridge_immediate --ros-args -p android_ip:=192.168.20.31
ros2 run phonebot_bridge phonebot_udp_bridge_immediate --ros-args -p android_ip:=192.168.20.21
(RL_env_3_12) hrc@hrc-Nitro-ANV15-51:/media/hrc/T7_UBUNTU_ONLY/android_humanoid_all_files/PhoneBot/Ros2_bridge$ 
ros2 run phonebot_bridge phonebot_example_motor_pub --ros-args -p hz:=2.0
ros2 run phonebot_bridge phonebot_example_motor_pub --ros-args -p hz:=100.0



order on raspberry pi zero 2w:
pkill -f cursor-server

echo 'alias activate_phonebot="source /opt/ros/humble/setup.bash && source ~/PhoneBot_3_10/bin/activate && export PYTHONPATH=\$PYTHONPATH:~/PhoneBot_3_10/lib/python3.10/site-packages"' >> ~/.bashrc

activate_phonebot


htop
watch -n 1 vcgencmd measure_temp

echo 'alias enable_ros2="source /opt/ros/humble/setup.bash"' >> ~/.bashrc
source ~/.bashrc


enable_ros2

export MAKEFLAGS="-j1"
colcon build --symlink-install --executor sequential



the raspberry pi 5: hrc-desktop
ssh hrc@192.168.20.11
2xxxxxxc!



to install on pi zero:
sudo apt install net-tools
sudo apt install network-manager


to delete the snapd:
sudo apt purge snapd -y

rm -rf ~/snap
sudo rm -rf /snap
sudo rm -rf /var/snap
sudo rm -rf /var/lib/snapd

ps aux | grep snap
which snap
which snapd


for pi zero v2:

ssh hrc@192.168.20.7
romela111111


for ubuntu 24 on pi zero 2w :
add this to the source :
Suites: noble noble-updates noble-backports
then sudo apt update && sudo apt install ros-dev-tools for ros2 installation 

hrc@HWdriverV2:~$ sudo nano /etc/NetworkManager/NetworkManager.conf 
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports/
Suites: noble noble-updates noble-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg




echo 'alias enable_ros2="source /opt/ros/jazzy/setup.bash"' >> ~/.bashrc
source ~/.bashrc


ssh key::
ssh-keygen -t ed25519 -C "houruochen@g.ucla.edu" -f ~/.ssh/hrc_raspberrypi_5
ls ~/.ssh
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/hrc_raspberrypi_5

cat ~/.ssh/hrc_raspberrypi_5.pub



this is only need for serverversion of ubuntu, not the gui version:


echo 'alias enable_git="eval \"\$(ssh-agent -s)\" && ssh-add ~/.ssh/hrc_raspberrypi_5 2>/dev/null"' >> ~/.bashrc
echo 'alias enable_git="eval \"\$(ssh-agent -s)\" && ssh-add ~/.ssh/hrc_raspberrypi_3 2>/dev/null"' >> ~/.bashrc
echo 'alias enable_git="eval \"\$(ssh-agent -s)\" && ssh-add ~/.ssh/hrc_pi_zero 2>/dev/null"' >> ~/.bashrc


shwo the memory more clearly:
sudo apt install smem
smem -t
smem -r



how to scp files , copy move files :
scp -r hrc@192.168.20.11:~/Documents/PhoneBot_HWdriver/ros2 ~/
hrc@192.168.20.11 is the pi 5 !!! 





for the usb delays :
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

Make it persistent (udev rule)
sudo nano /etc/udev/rules.d/99-usb-serial.rules
ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
sudo udevadm control --reload-rules
sudo udevadm trigger


to make sure the usb port access: 
sudo chown -R hrc /usr/local
sudo usermod -a -G dialout hrc
newgrp dialout



pip install . --break-system-packages
sudo cp 00-WestwoodRobotics.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm control --reload
sudo udevadm trigger


ros2 topic hz /phonebot/motor_state --window 50
ros2 topic hz /phonebot/motor_state_full --window 50

ros2 topic hz /phonebot/motor_cmd --window 50





source install/setup.bash
ros2 run phonebot_bridge_cpp phonebot_udp_bridge_immediate_cpp  --ros-args -p android_ip:=192.168.20.21
ros2 run phonebot_bridge_cpp phonebot_udp_bridge_immediate_cpp  --ros-args -p android_ip:=192.168.20.31
ros2 run phonebot_bridge_cpp phonebot_udp_bridge_immediate_cpp  --ros-args -p android_ip:=192.168.20.6



colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release




IP Address 	Device Name 	MAC Address
 	1	192.168.20.3	MOCAP-WINDOWS	30:9C:23:97:2A:EE
 	2	192.168.20.33	HWdriver	D8:3A:DD:42:26:91
 	3	192.168.20.34	radxa-zero	20:50:E7:D2:66:DE
 	4	192.168.20.15	hrc-Nitro-ANV15-51	C0:BF:BE:9D:88:DF
 	5	192.168.20.11	hrc-desktop	2C:CF:67:37:D6:58
 	6	192.168.20.2	hrc	30:F6:EF:C2:D9:40
 	7	192.168.20.7	HWdriverV2	88:A2:9E:08:DD:CB
 	8	192.168.20.14	hrc-raspberrypi-3B-plus	B8:27:EB:0D:A7:44




ssh hrc@192.168.20.14
2xxxxc!

nmcli dev wifi connect "RoMeLa_Apollo-5G" password "RoMeLa_Lab_UCLA"
nmcli dev wifi connect "RoMeLa_Apollo-2G" password "RoMeLa_Lab_UCLA"

nmcli dev wifi connect "romela_robocup_2G" password "RoMeLa_Lab_UCLA"
nmcli dev wifi connect "romela_robocup_5G1" password "RoMeLa_Lab_UCLA"

romela_robocup_5G2
romela_robocup_5G1
romela_robocup_2G



sudo apt install iw
iw list | grep -A 10 "Supported interface modes"




sudo nmcli device wifi rescan
nmcli device wifi list