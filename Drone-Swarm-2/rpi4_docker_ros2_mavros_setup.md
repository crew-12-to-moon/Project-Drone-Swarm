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
```
