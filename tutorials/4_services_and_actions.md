# Tutorial 4: Writing Services and Actions in ROS 2

You’ve used topics for one-way communication.
Now we’ll build **services** (request–response) and **actions** (long-running goals).

---

## 4.1 Writing a Simple Service Server

Create `my_first_pkg/service_server.py`:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

## 4.2 Writing a Service Client

Create `my_first_pkg/service_client.py`:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    future = node.send_request(2, 7)
    rclpy.spin_until_future_complete(node, future)
    response = future.result()
    node.get_logger().info(f'Result: {response.sum}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

---

## 4.3 Run It!

In one terminal:
```bash
ros2 run my_first_pkg service_server

```

In another:
```bash
ros2 run my_first_pkg service_client

```

You should see:
```bash
[Server] Incoming request: a=2, b=7
[Client] Result: 9

```

---

## 4.4 Intro to Actions

Actions are like advanced services:
- Allow feedback during execution
- Support cancellation
- Perfect for navigation or long-running goals

Example: `/turtle1/rotate_absolute` in `turtlesim`.

Try:
```bash
ros2 action list
ros2 action info /turtle1/rotate_absolute

```

---

## 4.5 Exercise

✅ Modify the AddTwoInts service to multiply instead.
✅ Explore an action client using `ros2 action send_goal`.

---

You now understand two-way communication in ROS 2: **Services and Actions.**
