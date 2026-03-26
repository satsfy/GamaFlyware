#  GamaFlyware

An Open-Source Simulation Platform for Autonomous Vision-Guided UAV Missions.

https://github.com/user-attachments/assets/35d8d9f1-912d-4cb9-a0b7-0b1c03272758

## About the System

The platform can support several features: 
- Navigating areas via Python code
- Autonomous landing in pads
- Simulatenous Localization and Mapping (SLAM)
- Surveying indoor or outdoor environments
- SLAM point cloud processing
- Live plotting of flight data
- Missions in several maps

The system is implemented in a [DockerHub image](https://hub.docker.com/r/renatobrittoaraujo/gamaflyware/tags) to be run on a Desktop, including all the tools, GUI and instalations. Should be simple to start using.

You can also read [my Software Engineering B.A. Capstone Project about GamaFlyware](BA_Thesis_about_GamaFlyware.pdf).


## Prerequisites

Make sure you have the following installed on your Linux Ubuntu 22.04 host:

* **VSCode and Contaier extension**

To install VSCode and the Container extension:

```bash
sudo snap install --classic code
```

After installing VSCode, launch it and open the Extensions Marketplace. Search for and install the **Dev Containers** extension (by Microsoft).

* **Docker & Docker Compose plugin**

```bash
sudo snap install --classic code
```

Then, open VSCode and install the **Dev Containers** extension from the Extensions Marketplace.

```bash
sudo apt-get update
sudo apt-get install docker.io docker-compose-plugin -y
```

* **NVIDIA drivers**

To enable GPU acceleration with Docker Compose, download and install the latest NVIDIA drivers for your GPU from the [official NVIDIA website](https://www.nvidia.com/Download/index.aspx). After installing the drivers, also install the NVIDIA Container Toolkit:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

> **Note:** Proper driver installation is required for GPU passthrough and CUDA support inside containers.

## Getting Started

1. **Clone this repository**:

```bash
git clone https://github.com/RenatoBrittoAraujo/GamaFlyware
```

## Pulling the Docker Image

This prebuilt image (\~40 GB) contains PX4-Autopilot, Gazebo Garden, ROS 2 Humble, and all dependencies:

```bash
sudo docker pull renatobrittoaraujo/gamaflyware:2
```

> **Note:** Download may take several minutes.

## Launching the Container

1. Remove or rename any existing container named `gamaflyware`.
2. Edit `docker-compose.yml` if you need to change RAM/CPU limits or adapt GPU settings for non-Ubuntu hosts.
3. Start in detached mode:

```bash
sudo docker compose up -d
```

## Allowing GUI Windows from Docker

On the host, permit the container to open windows (Gazebo, OpenCV GUIs, etc.):

```bash
xhost local:docker
```

## Running Your Code

### Option 1: Launch all terminals with VSCode extension

In VSCode, type `Ctrl+Shift+P` and select **Dev Containers: Restore and Reopen in Container**. This will automatically restore all recommended terminals for your workspace.

### Option 2: Manual Steps

1. **Open a shell in the container**

```bash
sudo docker exec -it gamaflyware bash
```
2. **Start PX4 SITL + Gazebo**

```bash
cd PX4-Autopilot
source /opt/ros/humble/setup.bash
make px4_sitl gz_x500
```
3. **Build & run your ROS 2 package**

```bash
cd ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select mission
ros2 run mission land
```
4. **Launch XRCE or MAVROS agent**

* **MAVROS:**

```bash
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://@127.0.0.1:14545
```
5. **Run QGroundControl**

```bash
./QGroundControl.AppImage
```
6. **Bridge Gazebo camera to ROS**

```bash
cd ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run ros_gz_image image_bridge /camera
```

## Troubleshooting & Tips

* **MAVROS “broadcast request timeout” warning**:

1. Install `ss` inside container:

```bash
sudo apt-get update && sudo apt-get install -y iproute2
```
2. List UDP ports:

```bash
ss -uln
```
3. Rerun MAVROS with correct port:

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:<YOUR_PORT>
```

* **Changing Gazebo world/model paths**:

Use models folder `gz` to insert your new models.
