# The Living Map — Phase 1 ROS 2/Gazebo Co-Simulation

## Correct system architecture

The **Outside Network Area is the Gateway itself**. It is not a separate cloud/network node.

Inside the GPS-denied Danger Zone:

```text
Writer
  |
  | ESP-NOW (local write)
  v
Beacon #5 -> #4 -> #3 -> #2 -> #1
                              |
                              | boundary to outside
                              v
                         OUTSIDE NETWORK AREA
                              = GATEWAY
                           /           \
                        LoRa           LoRa
                         /               \
                Command Post         Executor
```

Beacon #1 is physically dropped at the entrance **first**. The Writer then drops #2, #3, #4 and #5 as it explores. The beacons are both persistent memory nodes and **navigation breadcrumbs**.

### Communication rules

1. Writer does **not** directly contact the Gateway while exploring.
2. An event is written to the deepest/local beacon currently available.
3. That beacon forwards the event memory toward Beacon #1 using the ESP-NOW relay chain.
4. Beacon #1 is the only inside beacon connected to the Outside Network Area/Gateway.
5. The Gateway receives event information through Beacon #1 and translates it into an outside mission/alert.
6. Gateway sends separate LoRa messages to the Command Post and Executor.
7. Command Post and Executor never communicate directly.
8. When the Writer physically returns to the entrance, and only then, it can communicate directly with the Gateway for return synchronization.
9. Executor enters through Beacon #1 and follows the deployed beacon breadcrumb sequence toward the event location.
10. Executor sends its report only to the Gateway; Gateway forwards it to the Command Post.

## Phase 1 alignment

The challenge requires a Writer and Executor, at least two event types, beacon deployment, an Outside Network Area, and gateway-mediated communication. The specification also requires the Executor to navigate using inherited beacon information.

## Run locally

```bash
cd /workspace/tsyp_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tsyp_simulation scenario_launch.py
```

The Gazebo world uses local geometry; no Fuel model download is required for the building/hazards/beacons. The two robots are official TurtleBot3 Waffle Pi models with DiffDrive, odometry, laser scan, IMU, and camera sensors. The launch file uses per-robot SDF copies so their command and sensor topics do not collide.

## Run with Docker

From the directory containing `Dockerfile`:

```bash
xhost +local:docker
docker compose build
docker compose run --rm tsyp-simulation ros2 launch tsyp_simulation scenario_launch.py
```

If the host uses Wayland and the GUI does not open, run the container from an X11 session or keep `DISPLAY` and `XAUTHORITY` exported before starting the container.

To open a shell in the built image:

```bash
docker compose run --rm tsyp-simulation bash
```

Useful sensor checks from a second terminal:

```bash
docker compose run --rm tsyp-simulation ros2 topic echo /model/writer_robot/odom
docker compose run --rm tsyp-simulation ros2 topic echo /model/writer_robot/scan
docker compose run --rm tsyp-simulation ros2 topic echo /model/writer_robot/imu
```

## Important demo sequence

Watch for these messages in the terminal:

```text
PHYSICAL DROP: Beacon #1 ...
PHYSICAL DROP: Beacon #2 ...
PHYSICAL DROP: Beacon #3 ...
EVENT EVT-003 (VICTIM) detected -> written to Beacon #3 -> forwarded toward #1
[ESP-NOW Hop ...] BEACON-3 -> BEACON-2
[ESP-NOW Hop ...] BEACON-2 -> BEACON-1
[ESP-NOW Hop ...] BEACON-1 -> GATEWAY
Gateway dispatch -> Command Post + Executor via LoRa
WRITER RETURNED: direct Writer <-> Outside Network Gateway link now active
Executor reached Beacon #1 ...
Executor reached Beacon #2 ...
...
Executor report sent to Gateway
```
