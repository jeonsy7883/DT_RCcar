#!/bin/bash
set -e

echo "[1/6] locale 설정"
apt update
apt install -y locales
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

echo "[2/6] 준비 패키지 설치"
apt install -y software-properties-common curl
add-apt-repository universe -y

echo "[3/6] ROS2 저장소 키 등록"
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "[4/6] ROS2 apt 저장소 등록"
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list

echo "[5/6] ROS2 Humble + Nav2 설치 (시간 걸림)"
apt update
apt install -y ros-humble-desktop ros-humble-navigation2 ros-humble-nav2-bringup python3-colcon-common-extensions python3-rosdep

echo "[6/6] rosdep 초기화"
rosdep init || true
sudo -u "$SUDO_USER" rosdep update

if ! grep -q "source /opt/ros/humble/setup.bash" "/home/$SUDO_USER/.bashrc"; then
  echo "source /opt/ros/humble/setup.bash" >> "/home/$SUDO_USER/.bashrc"
fi

echo ""
echo "=== 설치 완료 ==="
source /opt/ros/humble/setup.bash
ros2 pkg list | grep nav2_bringup && echo "nav2_bringup 정상 확인됨"
