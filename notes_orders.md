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
echo 'alias enable_git="ssh-add ~/.ssh/hrc_raspberrypi_5 2>/dev/null"' >> ~/.bashrc
echo 'alias enable_git="ssh-add ~/.ssh/hrc_pi_zero 2>/dev/null"' >> ~/.bashrc


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


pip install . --break-system-packages
sudo cp 00-WestwoodRobotics.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm control --reload
sudo udevadm trigger


ros2 topic hz /phonebot/motor_state --window 50