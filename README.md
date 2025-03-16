# `digit_ros2` - ROS 2 Package for Agility Digit Robot

`digit_ros2` is a **ROS 2 Humble** package that provides a hardware interface for controlling the **Digit robot**. It enables communication with Digit's **WebSocket API** and **low-level control systems**.

---

## 📦 Installation

### 1️⃣ **Clone the Repository**
```bash
cd ~/ros2_ws/src
git clone https://github.com/purdue-tracelab/digit_ros2
```
### 2️⃣ Install the Package
digit_hardware package relies on websocket to send JSON message to request priviledge and set operation mode.

```pip install websocket-client```

### 3️⃣ Build the Workspace
``` bash
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Running the Simulation
### 1️⃣ Start the Agility Robotics Simulator

Before launching ROS 2, you need to start the Agility Robotics (AR) simulator.

```bash
cd /path/to/simulator/dir
./ar-control lowlevelapi_example.toml
```

This will launch the Agility Robotics simulator in low-level API mode, allowing ROS 2 to send control commands.
### 2️⃣ Launch the ROS 2 Simulation Package

Once the AR simulator is running, start the ROS 2 simulation package:

```bash
ros2 launch digit_launch simulation.launch.py
```

This starts all necessary ROS 2 nodes, including Digit’s hardware interface.

> ‼️ The package only requests for priviledge on initialization. You have to re-launch the project to get low level privilege if disconnected.