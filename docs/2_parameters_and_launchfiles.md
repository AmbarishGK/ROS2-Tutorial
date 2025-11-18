# Tutorial 2: Parameters and Launch Files

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
**Parameters** are node settings you can change without recompiling. **Launch files** start many nodes with configs in one command.

## Parameters
```bash
ros2 param list
ros2 param set /turtlesim background_r 255
ros2 param set /turtlesim background_g 100
```

## Launch multiple nodes
Use the provided example:
```bash
ros2 launch /home/ros/ros2_tutorial/examples/turtlesim_start.launch.py
```

### Command Breakdown
- `ros2 param` lets you inspect/change runtime settings.
- `ros2 launch` runs multi-node systems with parameters and remappings.

## Why this matters
Real robots run many nodes; launch files give you a reproducible recipe.

## How you’ll use it
You’ll bundle your systems into launch files to start everything consistently on any machine.

## Wrap‑up
- You tuned parameters live.
- You launched multiple nodes at once.
- You learned how to scale beyond one terminal per node.

**Next:** Tutorial 3 - Create a Package and Pub/Sub
