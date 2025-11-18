# Tutorial 4: Services and Actions

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
**Services** are synchronous request/response (like a function call). **Actions** are for long‑running goals with feedback (like navigation).

## Service example
Terminal A:
```bash
python3 /home/ros/ros2_tutorial/examples/add_two_ints_server.py
```
Terminal B:
```bash
python3 /home/ros/ros2_tutorial/examples/add_two_ints_client.py
```

### Code walkthrough
- Server: `create_service(AddTwoInts, 'add_two_ints', callback)`
- Client: waits for service, sends request, prints result.

## Actions (explore)
```bash
ros2 action list
ros2 action info /turtle1/rotate_absolute
```

## Why this matters
Some tasks need a clear request/response or progress feedback (e.g., move to pose).

## How you’ll use it
Use services to configure or trigger events; use actions for goals that take time.

## Wrap‑up
- You implemented a service server and client.
- You learned when to use actions vs services.
- You explored turtlesim’s action interface.

**Next:** Tutorial 5 - TF2 and RViz2
