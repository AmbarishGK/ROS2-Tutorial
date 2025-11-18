# Tutorial 1: Nodes, Topics, and Services (with Turtlesim)

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
A **node** is a running process. Nodes communicate using **topics** (pub/sub) and **services** (request/response). This separation lets you build modular robots where sensors, control, and UI are independent.

## Run a node (turtlesim)
```bash
ros2 run turtlesim turtlesim_node
```
Open a new terminal and list nodes:
```bash
ros2 node list
```

## Teleop node
```bash
ros2 run turtlesim turtle_teleop_key
```
Move the turtle with arrow keys.

### Command Breakdown
- `ros2 run PACKAGE EXECUTABLE` starts a node.
- `ros2 node list` shows running nodes.
- Two nodes are now running and communicating indirectly.


## Observe topics & services
```bash
ros2 topic list
ros2 topic echo /turtle1/pose
ros2 service list
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 't2'}"
```

## Why this matters
Topics decouple producers and consumers; services handle point-in-time operations (like spawning objects).

## How you’ll use it
Nearly every sensor publishes on a topic (e.g., `/scan`). Your code will subscribe to those or provide services.

## Wrap‑up
- You ran two nodes and saw them interact.
- You inspected topics and called a service.
- You understand node boundaries.

**Next:** Tutorial 2 - Parameters and Launch Files
