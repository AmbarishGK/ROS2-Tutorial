# Tutorial 6: URDF and Robot Description

Learn to represent your robot in 3D using URDF (Unified Robot Description Format).

## 6.1 What is URDF?
URDF describes the robot’s physical structure ; links, joints, and sensors ; in XML.

Example `simple_robot.urdf`:
```xml
<robot name="simple_bot">
  <link name="base_link"/>
  <joint name="base_to_camera" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>
  <link name="camera_link"/>
</robot>

```

## 6.2 Visualize URDF in RViz2
```bash
ros2 launch urdf_tutorial display.launch.py model:=simple_robot.urdf

```

## 6.3 Exercise
✅ Add a new link `lidar_link` offset forward by 0.1 m.
