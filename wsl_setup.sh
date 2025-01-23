# install ros2 humble
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
sudo apt-get install ros-humble-rosbridge-suite

# install pytrees

sudo apt install \
    ros-humble-py-trees \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-py-trees-ros

# rosdep setup
sudo rosdep init 
sudo rosdep update

# install gnome-screenshot to enable screen shots
sudo apt install gnome-screenshot

# install python packages
sudo apt install python3-pip
pip3 install -r requirements.txt

# source bash to run ros2 packages
echo "source ~/code/mini_platform/install/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# enable auto-fill for colcon
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# create Xauthority manually to use pyautogui
touch ~/.Xauthority
xauth add ${HOST}:0 . $(xxd -l 16 -p /dev/urandom)
xauth list

# configure mirrored networking mode for wsl: 
# https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking

# setup connection between fe and be
# setup startup execution of services

# 1. install rosbridge suite

# 2. create a .sh bash script in /usr/bin and put the command on startup into the file

# 	2.1 usr bin env bash
# 	2.2 source ros directories
# 	2.3 setup ROS_LOG_DIR
# 	2.4 start daemon (only needed in wsl)
# 	2.5 start launcher service node

# below is the command for the .sh file

# #!/usr/bin/env bash
# # To be placed in /usr/bin/
# # Launch the mini platform
# export ROS_LOG_DIR=~/code/mini_platform/src/backend/log

# source /opt/ros/humble/setup.bash
# source ~/code/mini_platform/install/setup.bash

# ros2 daemon start
# ros2 launch launch_files backend_launcher_launch.yaml

touch mini_platform_launch_script.sh
printf '!/usr/bin/env bash\n# To be placed in /usr/bin/\n# Launch the mini platform\nexport ROS_LOG_DIR=~/code/mini_platform/src/backend/log\nsource /opt/ros/humble/setup.bash\nsource ~/code/mini_platform/install/setup.bash\nros2 daemon start\nros2 launch launch_files backend_launcher_launch.yaml' >> mini_platform_launch_script.sh
sudo mv mini_platform_launch_script.sh /usr/bin

# 3. grant execution permission to the .sh script by chmod +x .sh

# 4. create a .service file in /usr/lib/systemd/system 
# 	3.1 configure User
# 	3.2 configure startup executable

# below is the .service file
# [Unit]
# Description=Robot launch script

# [Service]
# User=$USER
# ExecStart=/usr/bin/mini_platform_launch_script.sh

# [Install]
# WantedBy=multi-user.target

# 5. enable startup service by sudo systemctl enable .service

touch mini_platform_launch.service
printf '[Unit]\nDescription=Robot launch script\n[Service]\nUser=%s\nExecStart=/usr/bin/mini_platform_launch_script.sh\n[Install]\nWantedBy=multi-user.target' $USER >> mini_platform_launch.service
sudo mv mini_platform_launch.service /usr/lib/systemd/system
sudo systemctl enable mini_platform_launch.service