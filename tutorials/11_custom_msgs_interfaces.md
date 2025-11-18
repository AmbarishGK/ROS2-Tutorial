# Tutorial 11: Custom Messages and Interfaces

Create your own message and service types.

## 11.1 Create Message Definition
In `my_interfaces/msg/Num.msg`:
```plaintext
int64 num

```

## 11.2 Build and Use
```bash
ros2 pkg create my_interfaces --build-type ament_cmake
colcon build

```

## 11.3 Exercise
✅ Create a custom service that squares a number.
