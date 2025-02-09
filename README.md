# DoubleVision and ROS2
Get started with the Robotic Operating System (ROS) framework used in the Spy Turtle - Double Vision project. 

## Run ROS2 on your local host
#### The following instructions worked great on a Macbook Pro using Terminal and Docker Desktop. You may need to slightly adapt it to fit your local system's particulars!

### SETUP DOCKER CONTAINER
1. List all docker containers and inspect their image values
```bash
docker ps -a
```
2. You need ubuntu 22.04 for Ros2 humble
```bash
docker pull ubuntu:22.04
```
3. Open an interactive shell inside a new container with the ubuntu 2204 image
```bash
docker run -it --name ros2204_container ubuntu:22.04
```
4. Once inside your container update and install locales
```bash
apt update && apt install locales
```
5. Set your container's locale to US UTF-8
```bash
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
6. Verify your locale settings
```bash
locale
```
### ADD the ROS 2 REPO
1. First ensure that Ubuntu Universe repository is enabled
```bash
apt install software-properties-common
add-apt-repository universe
```
2. Now you're ready to add the ROS 2 GPG key
```bash
apt update && apt install curl -y
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```
3. Finally, add the ROS 2 repository to your sources list
```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### INSTALL ROS 2 Humble
1. Install common packages
```bash
apt update && apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools
```
2. Install Ubuntu 22.04 specific packages
```bash
apt install -y \
   python3-flake8-blind-except \
   python3-flake8-builtins \
   python3-flake8-class-newline \
   python3-flake8-comprehensions \
   python3-flake8-deprecated \
   python3-flake8-import-order \
   python3-flake8-quotes \
   python3-pytest-repeat \
   python3-pytest-rerunfailures
```

### BUILD ROS 2 CODE
1. Create a workspace and clone all repos
```bash
mkdir -p ~/ros2_humble/src
cd ~/ros2_humble/
vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
```
2. Install dependencies using rosdep
```bash
apt upgrade
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"
```
3. Ensure you're working in a fresh environment without a sourced ROS 2
```bash
printenv | grep -i ROS
```
4. Build the code! This might take a while...
```bash
cd ~/ros2_humble/
colcon build --symlink-install
```

### RUN SOME EXAMPLES
1. Setup your environment by sourcing the setup script
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
. ~/ros2_humble/install/local_setup.bash
```
2. Try the Publisher/Listener example!
Start the talker
```bash
ros2 run demo_nodes_cpp talker
```
In another terminal, source the setup script and start the listener!
```bash
. ~/ros2_humble/install/local_setup.bash
ros2 run demo_nodes_py listener
```
3. Continue with other [tutorials and demos](https://docs.ros.org/en/humble/Tutorials.html)!
