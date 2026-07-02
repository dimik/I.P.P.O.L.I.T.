#!/bin/bash
# Install ROS 2 Jazzy on Radxa Dragon Q6A (Ubuntu 24.04 noble, arm64).
# Run as a NORMAL user with passwordless sudo (e.g. `radxa`):  bash install_ros2.sh
# System changes use sudo explicitly; rosdep update + .bashrc run as the invoking user.
set -euo pipefail

ARCH="$(dpkg --print-architecture)"          # arm64 on the Q6A
CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"   # noble

echo "=== Setting locale ==="
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "=== Enabling universe + ROS 2 apt repository ==="
sudo apt-get install -y curl software-properties-common
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "=== Installing ROS 2 Jazzy base + Nav2 ==="
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-base \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-msgs \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-tf2-ros \
  python3-colcon-common-extensions \
  python3-rosdep

echo "=== Initialising rosdep ==="
sudo rosdep init || true
rosdep update

echo "=== Adding ROS setup to ~/.bashrc ==="
grep -qxF "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc" \
  || echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"

echo "=== DONE. ROS 2 Jazzy installed: $(ls -d /opt/ros/jazzy) ==="
echo "Run: source /opt/ros/jazzy/setup.bash"
