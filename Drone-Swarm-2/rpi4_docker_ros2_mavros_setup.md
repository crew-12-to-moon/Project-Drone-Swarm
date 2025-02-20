Docker-based ROS 2 Humble & MAVROS Setup Guide 
===============================================
### Mavros version: 
Below are the steps to install Docker, run a ROS 2 Humble container, check ROS 2 functionality, install MAVROS inside the container, and finally save your Docker container progress.
## 1. Install Docker
-----------------
### Update the system
```bash
sudo apt update && sudo apt upgrade -y
```
### Download and install Docker
```bash
curl -sSL https://get.docker.com | sh
```
### Add user to the Docker group (to run Docker without sudo)
```bash
sudo usermod -aG docker $USER
```
> **Note:** Restart your terminal or log out & log in for the group changes to take effect.
### Verify Docker installation
```bash
sudo docker run hello-world
```
If Docker is installed correctly, you should see a message confirming its successful execution.
## 2. Pull and Run ROS 2 Humble in Docker
---------------------------------------
### Pull the ROS 2 Humble base image
```bash
sudo docker pull ros:humble-ros-core
```
### Run the ROS 2 container interactively
```bash
sudo docker run -it ros:humble-ros-core
```
This starts a new container with ROS 2 Humble.
## 3. Check if ROS 2 Humble is Working
------------------------------------
### Publish a test message on a ROS 2 topic
```bash
ros2 topic pub /example_topic std_msgs/msg/String "data: 'Hello ROS 2'"
```
### Echo the published topic to verify it
```bash
ros2 topic echo /example_topic
```
If ROS 2 is working correctly, you should see "Hello ROS 2" being printed.
## 4. Install MAVROS inside Docker
-------------------------------
### Update package list
```bash
apt update
```
### Install MAVROS and MAVROS extras
```bash
apt install -y \
    ros-humble-mavros \
    ros-humble-mavros-extras
```
### If GeographicLib error comes
Run the following code to install one last dependency
```bash
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

```

### Source ROS 2 setup script automatically on every new shell session
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```
### Verify MAVROS installation
```bash
ros2 pkg list | grep mavros
```
If installed successfully, `mavros` should appear in the list.
## 5. Save Docker Container Progress
----------------------------------
### Exit the running container
```bash
exit
```
### List all Docker containers (including stopped ones)
```bash
docker ps -a
```
> **Note:** The latest container (top entry) is likely the one you need.
### Commit the container with a custom name
```bash
docker commit <container_id> <container_name>
```
Replace `<container_id>` with the actual ID from `docker ps -a` and `<container_name>` with a meaningful name (e.g., `ros2_mavros_container`).
### Run the saved container
```bash
docker run -it <container_name>
```

This allows you to resume work inside the saved container without losing progress.


# ArduPilot SITL and MAVROS Setup

Install and run the ArduPilot SITL simulation for ArduCopter and launch MAVROS using ROS2. It also explains how to run multiple simulation instances. All commands are provided as-is with added comments for clarity.
---
## Prerequisites
- **Operating System:** Ubuntu/Debian (or Windows with WSL)
- **ROS2:** Installed and properly configured
- **MAVROS:** Installed in your ROS2 workspace
---
## 1. System Update and Dependency Installation
Update your package list and install Git, Python3, and the required development libraries and packages.
```bash
apt-get update
apt-get install git python3 python3-dev python3-pip python3-numpy python3-matplotlib
python3 -m pip install MAVProxy
```

## 2. Cloning the ArduPilot Repository

Clone the ArduPilot repository from GitHub and initialize all necessary submodules.

```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git checkout stable-copter-4.5.7
git submodule update --init --recursive
./waf configure --board sitl
./waf build
```

## 3. Running the ArduCopter Simulation

Navigate to the ArduCopter directory and start the SITL simulation with console and map displays.

```bash
cd ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -w --console --map
```
and if you get this error: "-bash: sim_vehicle.py: command not found", So to execute sim_vehicle.py from any location, add its directory to your PATH:

```bash
export PATH=$PATH:$HOME/ardupilot/Tools/autotest
```
After adding this line, reload your shell configuration:
```bash
source ~/.bashrc
```
and after all this, if you still end up with this error: "/usr/bin/env: ‘python’: No such file or directory", create a symlink:
```bash
ln -s /usr/bin/python3 /usr/bin/python
```
Now run the sitl simulation!

## 4. Running MAVROS with ROS2

After starting the simulation, launch MAVROS to interface with the SITL. Open a new terminal and run:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

This command connects MAVROS to the simulated flight controller using the specified UDP connection parameters.

To verify that topics are published (e.g., local position), run:

```bash
ros2 topic echo /mavros/local_position/pose
```


### Start Multiple SITL Instances

For instance 0:

```bash
sim_vehicle.py -v ArduCopter -w --console --map -f quad -I 0
```

For instance 1:

```bash
sim_vehicle.py -v ArduCopter -w --console --map -f quad -I 1
```

## Launch MAVROS for Each Instance

Launch MAVROS for each simulation instance with the appropriate namespace and connection ports (ensure these match the ports shown in the SITL logs):

### For Drone 1:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 namespace:=/drone1
```

### For Drone 2:

```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14560@14557 namespace:=/drone2
```
Note: Verify that the port numbers (14550@14555 and 14560@14557) are consistent with the logs provided by the ArduPilot SITL for each instance.
Footer

## communicate via ROS DOMAIN ID for ROS2:
### For ROS DOMAIN ID to work across different systems, you have to enable same DDS version implemented on all the systems (here we use cyclone dds)

Install Cyclone DDS on your Raspberry Pi:
Try updating your package list and installing the Cyclone DDS package for ROS2 Humble:

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

Set the Environment Variable:
Ensure you force ROS2 to use Cyclone DDS by running:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Re-source Your ROS2 Setup:
Make sure you have sourced your ROS2 Humble setup script:

```bash
source /opt/ros/humble/setup.bash
```
