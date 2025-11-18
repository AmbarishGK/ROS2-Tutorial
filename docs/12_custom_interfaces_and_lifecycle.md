# Tutorial 12: Custom Interfaces and Lifecycle Nodes

> **Environment Note**  
> - Make sure the course container is running (see Tutorial 0).  
> - Open a new terminal in the same container with: `docker exec -it ros2course bash`  
> - In each new terminal: `source /opt/ros/humble/setup.bash`


## Concept Overview
Custom **messages/services** define your robot’s APIs. **Lifecycle nodes** formalize startup/teardown states (configure → activate → deactivate).

## Custom message
Create package `my_interfaces`, add `msg/Num.msg`:
```
int64 num
```
Build and use it in a pub/sub node.

## Lifecycle flow
```bash
ros2 run lifecycle demos_lifecycle_talker
ros2 lifecycle list /talker
ros2 lifecycle set /talker configure
ros2 lifecycle set /talker activate
```

## Why this matters
Clean, explicit interfaces make systems reusable; lifecycle makes startup predictable.

## How you’ll use it
Define messages once, reuse everywhere; gate publishers/subscribers with lifecycle state.

## Wrap‑up
- You created a custom interface.
- You practiced lifecycle transitions.
- You prepared nodes for real deployments.

**Next:** Tutorial 13 - Multi‑Robot & Networking
