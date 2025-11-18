# Tutorial 12: Lifecycle and Composition Nodes

Lifecycle nodes manage states like unconfigured, inactive, active, finalized.

## 12.1 Lifecycle Example
```bash
ros2 run lifecycle demos_lifecycle_talker
ros2 lifecycle list /talker
ros2 lifecycle set /talker configure
ros2 lifecycle set /talker activate

```

## 12.2 Composition Example
Combine nodes in a single process:
```bash
ros2 run rclcpp_components component_container

```

## 12.3 Exercise
✅ Write a lifecycle node that logs when activated.
