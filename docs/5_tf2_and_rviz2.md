# Tutorial 5: TF2 and RViz2 Visualization

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
**TF2** tracks coordinate frames so nodes agree on where things are. **RViz2** lets you visualize data and frames.

## Launch demo
```bash
ros2 launch /home/ros/ros2_tutorial/examples/tf_demo.launch.py
```
Then open RViz2 and add the **TF** display.

## Inspect transforms
```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

## Why this matters
Navigation, manipulation, and perception all depend on consistent frames.

## How you’ll use it
You’ll broadcast static and dynamic transforms so all nodes share a common spatial view.

## Wrap‑up
- You launched a TF demo and viewed frames.
- You inspected a transform on the CLI.
- You used RViz2 for visualization.

**Next:** Tutorial 6 - URDF and Robot Description
