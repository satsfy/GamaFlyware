# GamaFlyware

An Open-Source Simulation Platform for Autonomous Vision-Guided UAV Missions.

https://github.com/user-attachments/assets/35d8d9f1-912d-4cb9-a0b7-0b1c03272758

## What is GamaFlyware

GamaFlyware is a complete plug and play sandbox for easy UAV (drone) mission simulation. It includes many more features such as:

- 44 worlds to use right out of the box. You can customize your own
- Sample missions for you to fork, including computer vision based missions
- Support for Python mission script creation.
- Expose telemetry and local control through ROS 2
- Programmatic drone navigation
- Autonomous landing in pads
- Surveying indoor or outdoor environments
- QGroundControl ground control station
- Simulatenous Localization and Mapping (SLAM) using ORB_SLAM3
- Simple point cloud processing samples
- Customizable GUI for live plotting for flight data
- Bridge camera images from the simulation into ROS, or into local environment

You can also read [my Software Engineering B.A. Capstone Project about GamaFlyware](BA_Thesis_about_GamaFlyware.pdf).

## System Overview

This platform was designed to be used inside a VSCode Dev Container for productivity. The system comes prepared for autocomplete, syntax highlighting and all necessary libraries installed. It runs inside a [DockerHub image](https://hub.docker.com/r/renatobrittoaraujo/gamaflyware/tags), including all the tools and GUI. Should be easy to get up and running.

Tech Stack:

- [PX4 Autopilot]
- [Gazebo]
- [ROS 2 Humble]
- [MAVROS]
- [QGroundControl]
- [ORB-SLAM3] - Computer Vision
- Python libraries: rclpy (for ROS 2 Humble), OpenCV, YOLO, and cv_bridge.

The docker compose working directory is `/sitl`, it runs a privileged container with large memory / CPU limits, X11 socket passthrough for GUI apps and optional NVIDIA GPU reservation.

## Repo Stucture

| Path                     | Contents                                                                               |
| ------------------------ | -------------------------------------------------------------------------------------- |
| `.vscode`                | VSCode configuration including automatic command to lauch all 6 gamaflyware programs.  |
| `PX4-Autopilot/`         | Vendored PX4 source tree, with build/log/install artifacts already present.            |
| `gz/`                    | A modified copy of PX4 Gazebo models/worlds.                                           |
| `ORB_SLAM3_files/`       | Vendored ORB-SLAM3 plus dependencies like Pangolin, OpenCV, Eigen, and Catch2.         |
| `ws/`                    | Main ROS 2 workspace, containing the custom `mission` package and Python requirements. |
| `px4_msgs_ws/`           | Auxiliary workspace for `px4_msgs` and `px4_ros_com`.                                  |
| `docs/`                  | Docs and my bachelor thesis PDF.                                                       |
| `contrib/`               | Shell helpers.                                                                         |
| `docker-compose.yml`     | Main container launch config. Uses the published image instead of building locally.    |
| `run_px4_sitl.sh`        | Starts PX4 SITL against a selected Gazebo world using `gz_x500_depth`.                 |
| `run_mavros_node.sh`     | Starts MAVROS with a fixed UDP URL.                                                    |
| `run_image_bridge.sh`    | Bridges the Gazebo camera into ROS with `ros_gz_image`.                                |
| `run_orbslam3.sh`        | Launches the ORB-SLAM3 webcam wrapper.                                                 |
| `run_qground_control.sh` | Attempts to run QGroundControl as `qgcuser`.                                           |

## Requisites

Use a Linux host, preferably Ubuntu 22.04 (which is the only tested environment).

### Install [Docker Compose](https://docs.docker.com/compose/install/linux/).

```
sudo apt-get update
sudo apt-get install docker-compose-plugin
```

## Install VSCode

```bash
sudo snap install --classic code
```

## Install VSCode Contaier Extension

After installing VSCode, launch it and open the Extensions Marketplace. Search for and install the **Dev Containers** extension (by Microsoft).

```bash
sudo snap install --classic code
```

### (optional) Install NVIDIA drivers

To enable GPU acceleration with Docker Compose, download and install the latest NVIDIA drivers for your GPU from the [official NVIDIA website](https://www.nvidia.com/Download/index.aspx). After installing the drivers, also install the NVIDIA Container Toolkit:

```bash
sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker
```

> **Note:** Running without GPU acceleration is slow. With GPU acceleration, it becomes smooth. Your choice. Proper driver installation is required for GPU passthrough and CUDA support inside containers.

### Pull the Docker Image

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

Allow GUI Windows from Docker: On the host, permit the container to open windows (Gazebo, OpenCV GUIs, etc.):

```bash
xhost local:docker
```

## Runtime

The runtime stack looks like this:

1. A prebuilt Docker image provides the heavy environment.
2. PX4 SITL runs inside the container against Gazebo (`gz_x500_depth`).
3. Gazebo worlds and models come from the large `gz/` directory.
4. MAVROS provides ROS-facing state, setpoint, arming, and mode control.
5. `ros_gz_image` bridges the Gazebo camera stream into ROS as an image topic.
6. The `mission` ROS 2 Python package runs mission logic.
7. Mission scripts use hardcoded waypoint state machines plus optional YOLO and QR detection.
8. ORB-SLAM3 can be launched separately and some scripts contain SLAM/GPS fusion or feedback logic.
9. QGroundControl is expected to be available in the container workflow for monitoring.

### Option 1: Launch all terminals with VSCode extension

In VSCode, type `Ctrl+Shift+P` and select **Dev Containers: Restore and Reopen in Container**. This will automatically restore all recommended terminals for your workspace.

Note that it also opens the optional side processes QGroundControl and ORB_SLAM3 for convenience.

### Option 2: Manual Steps

1. **Open a shell in the container**

```bash
sudo docker exec -it gamaflyware bash
```

2. **Start PX4 SITL + Gazebo**

```bash
./run_px4_sitl.sh lawn # you can easily choose any other simulation world you like by pressing <TAB>
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

- **MAVROS:**

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

## Missions

Package `mission` is a ROS 2 Python package containing sample mission scripts. Each script is a standalone mission.

The custom code lives in:

```bash
ws/src/mission
```

Run with:

```bash
ros2 run mission <module_name>
```

- `ws/src/mission/mission/land.py`
- `ws/src/mission/mission/triangle.py`
- `ws/src/mission/mission/walls.py`
- `ws/src/mission/mission/explore_indoor5.py`
- `ws/src/mission/mission/slam_hitl.py`

There are really two mission-code styles in this package.

### 1. Small, focused utilities

These are the easiest scripts to understand:

- `takeoff.py`: simple MAVROS offboard takeoff example.
- `arm_takeoff.py`: raw `pymavlink` force-arm and takeoff helper.
- `get_image_feed.py` and `img_sub.py`: subscribe to the camera topic and show frames in OpenCV.
- `imu_reader.py` and `raw_imu_reader.py`: plot IMU data with matplotlib.
- `monitor_mavlink.py`: print MAVLink messages after heartbeat.
- `msg_pub.py`: publish synthetic `LOCAL_POSITION_NED` messages.
- `post_hitl.py`: send `HIL_GPS` while adjusting coordinates from the keyboard.
- `qr_code_test.py`: local QR-code helper.
- `attitude.py`: tiny helper class.

### 2. Large, monolithic mission scripts

These are the core of the project:

- `land.py`
- `triangle.py`
- `walls.py`
- `explore_indoor5.py`
- `slam_hitl.py`
- `detecta_qrcodes_missao.py`
- `fase3_script.py`
- `ros_config.py`

These files combine several responsibilities at once:

- ROS node setup,
- state tracking,
- setpoint publishing,
- MAVROS service calls,
- plotting,
- keyboard runtime control,
- SLAM/GPS mode switching,
- camera callbacks,
- YOLO inference,
- QR-code processing,
- and finite-state mission logic.

## Vision, SLAM, And Mission Logic

### Vision

The project includes a local YOLO model:

```bash
ws/src/mission/mission/edra_colab_model.pt
```

Multiple mission scripts load it with Ultralytics YOLO and use it to detect bases or landing pads in the camera feed.

QR codes are handled through OpenCV's `QRCodeDetector`.

### SLAM

ORB-SLAM3 is vendored in the repo and launched through:

```bash
ORB_SLAM3_files/ORB_SLAM3/webcam/run.sh
```

That wrapper runs:

```bash
./webcam ../Vocabulary/ORBvoc.txt ./config_px4_sitl.yaml
```

Several mission files contain logic for switching between GPS-guided and SLAM-guided behavior, including modes such as:

- `GPS_MANUAL`
- `SLAM_MANUAL`
- `WITH_SLAM_RET`
- `CALIB_SLAM_MANUAL`
- `NONE`

or in `slam_hitl.py`:

- `GPS_MANUAL`
- `SLAM_MANUAL`
- `FEEDBACK`
- `NONE`

The larger scripts also contain live plotting support for SLAM, GPS, and setpoint trajectories, though plotting and keyboard control are often disabled by default.

### Mission styles

#### `triangle.py`

This appears to be a waypoint mission over three hardcoded base positions. The file still contains QR-code and YOLO-related machinery, but the active state machine at the bottom is simpler than the scaffolding suggests: it mostly arms, climbs, flies to the next base, confirms arrival from odometry, marks the base visited, and repeats.

#### `walls.py`

This is structurally similar to `triangle.py`, but the hardcoded coordinates define four wall-adjacent target points with specific yaw values and a high mission altitude. It also carries the same vision and QR-code helper logic even though the active state machine is still mostly waypoint-driven.

#### `explore_indoor5.py`

This mission builds a zig-zag grid of indoor waypoints and walks the drone through them. It looks like an exploration / coverage mission over a fixed map using hardcoded coordinates.

#### `land.py`

`land.py` is the most self-contained and distinctive mission file. Instead of subclassing the older `NoFase3` pattern, it defines its own internal classes for state, camera, control, and the mission. The mission scans a hardcoded waypoint grid, looks for landing-pad detections, centers over the closest detected pad using image-space error, and then performs a staged descent. It also contains MAVLink `vision_position_estimate` publishing and a richer plotting stack.

#### `slam_hitl.py`

This looks like an older or alternate generation of the mission architecture focused on SLAM/HITL behavior, manual feedback modes, and QR/base logic.

#### `detecta_qrcodes_missao.py` + `fase3_script.py` + `ros_config.py`

This looks like an earlier, more decomposed version of the mission framework where:

- `ros_config.py` holds the base ROS configuration,
- `fase3_script.py` defines a reusable finite-state machine shell,
- `detecta_qrcodes_missao.py` specializes it for vision / QR-based missions.

## Hardcoded Nature Of The Missions

A lot of the mission behavior is encoded directly in Python:

- waypoint coordinates,
- takeoff heights,
- state transitions,
- tolerances,
- YOLO confidence thresholds,
- altitude approach increments,
- SLAM/GPS behavior toggles,
- and keyboard-driven runtime adjustments.

This makes the repo very hackable for experiments, but it also means the code is not heavily parameterized or abstracted into reusable libraries.

## Worlds, Models, And Assets

The `gz/` directory is huge and important. It looks like a modified copy of PX4's Gazebo models/worlds repository, and the README mentions it is maintained as a subtree.

Notable observations:

- 51 world files are present in `gz/worlds/`.
- 521 top-level model directories are present in `gz/models/`.
- This includes stock-looking worlds plus custom / mission-relevant ones such as `walls`, `indoor5`, `indoor4`, `cbr_edra`, and others.

This folder is one of the reasons the repository is large.

## Troubleshooting & Tips

### MAVROS “broadcast request timeout” warning:

```bash
# Install `ss` inside container:
sudo apt-get update && sudo apt-get install -y iproute2

# List UDP ports:
ss -uln

# Rerun MAVROS with correct port:
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:<YOUR_PORT>
```

### Changing Gazebo world/model paths:

Use models folder `gz` to insert your new models.

### Configuring Display

Set the right display with:

```bash
export DISPLAY=:0 # in ubuntu, check what `echo $DISPLAY` spits out outside the repo and paste here
```

### (optional) Building deps

You need the following repos in `px4_msgs_ws` folder to build the ROS 2 messages and communication bridge:

- px4_msgs in branch release/1.15
- px4_ros_com in branch main

These need to be compiled with `colcon` so `ws` can use it for communication with PX4-Autopilot.

### Updating upstream

The `gz` folder is the remote repo https://github.com/PX4/PX4-gazebo-models/ with modifications I included. To update to upstream master, run:

```
git fetch gz-upstream
git subtree pull --prefix=gz gz-upstream main --squash
```

### Running QGroundControl

```bash
su qgcuser;
# now as user 'qgcuser'
./QGroundControl.AppImage
```

In case of failures, run `xhost +local:` in your host machine.
Then run:

```bash
su - qgcuser
export DISPLAY=:1 # here you can also try putting ':0' instead
cd /sitl
./QGroundControl.AppImage
```

This command should open QGroundControl from inside the container.

### Use git subtree for adding an upstream tracked repo with git support for local modifications

- `gz/` tracked as a **normal folder** in the main repo
- no separate checkout for collaborators
- ability to **pull upstream updates into just `gz/`**
- ability to keep your own local files/changes inside `gz/`

What you **cannot** do is keep `gz/.git` there and also have the parent repo track `gz/` as an ordinary directory. With that `.git` directory present, `gz` is a nested repo, not normal contents.

```bash
# Back it up first.
mv gz gz_backup

#Add the upstream remote to the parent repo. From the **main repo root**:
# If the remote already exists, skip the add.
git remote add gz-upstream <URL-OF-GZ-UPSTREAM>
git fetch gz-upstream

# Import `gz/` as a subtree. This creates `gz/` as a normal tracked folder in the parent repo:
git subtree add --prefix=gz gz-upstream <branch> --squash

#Now compare your old `gz_backup/` with the new subtree-imported `gz/`. Copy changes back (caution):
rsync -a --exclude=.git gz_backup/ gz/

# Then inspect carefully. Commit those local customizations as a separate commit. Then remove the backup when satisfied:
rm -rf gz_backup

# To update upstream changes:
git fetch gz-upstream
git subtree pull --prefix=gz gz-upstream <branch> --squash
# You will be able to resolve conflicts like a normal merge.
```
