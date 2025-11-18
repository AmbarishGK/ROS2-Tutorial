## Tutorial 2: Practical Tools for Real Systems

You now know how to run nodes. But in a real system, you'd have 10-20 nodes, each with its own settings. This tutorial covers the tools that make that complexity manageable.

---

### 2.1 What are Parameters? (The "Settings" for Your Nodes)

**Parameters** are the configurable settings for a node. They allow you to change a node's behavior without rewriting and recompiling the code.

**Why they are crucial:**
* **Tuning:** You can "tune" values (like motor speed or sensor sensitivity) while a node is running.
* **Reusability:** You can write one generic node (e.g., `camera_driver`) and use it on many different robots by simply giving it different parameter files (one for a USB cam, one for a depth cam, etc.).

#### 1. Inspecting Parameters
You can `list`, `get`, and `set` parameters on a live node. Let's use our running `turtlesim` node.

```bash
# In Terminal 3 (with turtlesim running)
# See all nodes and their available parameters
ros2 param list
```
    You will see `background_b`, `background_g`, and `background_r`.

2.  **Set Parameters:** Let's change the background color.
    ```bash
    # In Terminal 3
    ros2 param set /turtlesim background_r 255
    ros2 param set /turtlesim background_g 100
    ```
    The simulator's background color will change immediately. This is invaluable for tuning things like control gains or sensor thresholds.

---

### 2.2 What are Launch Files?

So far, you've been running your nodes using `ros2 run`. This is great for testing a single node, but it has a massive problem: **it doesn't scale.**

Think about the simple `turtlesim` example. You had to open *at least* two terminals:
1.  One for `ros2 run turtlesim turtlesim_node`
2.  One for `ros2 run turtlesim turtle_teleop_key`

Now, imagine a real robot. You might have:
* A node for the LiDAR
* A node for the camera
* A node for the wheel motors
* A node for the IMU (motion sensor)
* A node for the navigation logic
* A node for the main "brain"

Are you going to open 6, 10, or 20 terminals and run `ros2 run` in every single one, *every single time* you want to start your robot?

That would be a nightmare.

### The Solution: Launch Files

A **Launch File** is a single script that describes your *entire* system. Instead of running 10 `ros2 run` commands, you run **one** `ros2 launch` command.

This one command will:
1.  **Start** all your nodes at once.
2.  **Configure** them by loading their parameters (like from those YAML files).
3.  **Connect** them by remapping topics.
4.  **Organize** them by putting them in namespaces.

In short, a launch file takes your complex, multi-node system and makes it as easy to start as a single program. It's the standard, professional way to run any ROS 2 application that's more than a single test node.
**Example Launch File:** Create a file named `turtlesim_start.launch.py`.
  ```python
    # turtlesim_start.launch.py
    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='turtlesim',
                executable='turtlesim_node',
                name='sim'
            ),
            Node(
                package='turtlesim',
                executable='turtle_teleop_key',
                name='teleop',
                prefix='xterm -e' # Starts this node in a new xterm window
            )
        ])
  ```
    *(You may need to install xterm: `sudo apt install xterm`)*

4.  **Run the Launch File:** Close your other terminals and run this in a new one:
    ```bash
    source /opt/ros/humble/setup.bash
    ros2 launch ./turtlesim_start.launch.py
    ```
    Both the simulator and the keyboard controller will start from this single command. This is the standard way to run ROS 2 applications.
