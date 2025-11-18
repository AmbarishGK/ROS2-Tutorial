# Tutorial 3: Creating a ROS 2 Package and Publisher–Subscriber Nodes

You now know how to run prebuilt nodes and launch files.
Let’s take the next step **create your own node** in Python.

---

## 3.1 Creating a New Package

1️⃣ In your workspace (inside `/home/ros/ros2_ws`):
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_pkg --dependencies rclpy std_msgs

```

This command:
- Creates the folder `my_first_pkg/`
- Adds boilerplate files for `setup.py`, `package.xml`, etc.
- Declares dependencies on `rclpy` and `std_msgs`.

---

## 3.2 Writing the Publisher Node

Open `my_first_pkg/my_first_pkg/publisher_member_function.py` and add:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

## 3.3 Writing the Subscriber Node

Create `my_first_pkg/my_first_pkg/subscriber_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

## 3.4 Build and Run

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

```

### Run in two terminals:
```bash
ros2 run my_first_pkg publisher_member_function

```
and
```bash
ros2 run my_first_pkg subscriber_member_function

```

You’ll see logs like:
```bash
[INFO] Publishing: "Hello ROS 2: 0"
[INFO] I heard: "Hello ROS 2: 0"

```

---

## 3.5 Exercise

✅ Modify the publisher to publish the system time every second.
✅ Change the topic name and observe how the subscriber must match it.

---

**You now know how to write your own ROS 2 nodes!**
