# Tutorial 11: RQT, RViz, and TF Debugging

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
Use **RQT** GUIs and **RViz** to diagnose graphs, plots, logs, and transforms quickly.

## Tools
```bash
rqt_graph
rqt_plot
ros2 run tf2_tools view_frames
rviz2
```

### Tips
- `rqt_graph` shows node–topic connections.
- `rqt_plot` plots topic values over time.
- `view_frames` generates a TF tree pdf.

## Why this matters
Visual tools reduce guesswork and expose misconfigurations.

## How you’ll use it
You’ll use these constantly when integrating teams’ nodes and params.

## Wrap‑up
- You can inspect connections and signals.
- You can visualize TF trees and displays.
- You can spot mismatched topics quickly.

**Next:** Tutorial 12 - Custom Interfaces & Lifecycle
