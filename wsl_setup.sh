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
echo "source /home/xcao/code/mini_platform/install/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# enable auto-fill for colcon
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# create Xauthority manually to use pyautogui
touch ~/.Xauthority
xauth add ${HOST}:0 . $(xxd -l 16 -p /dev/urandom)
xauth list

# configure mirrored networking mode for wsl: 
# https://learn.microsoft.com/en-us/windows/wsl/networking#mirrored-mode-networking