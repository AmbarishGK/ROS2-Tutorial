# Your Guide to ROS 2 Humble

Let's dive into the core ideas of ROS 2. We'll use the classic `turtlesim` to make this stuff visual.

## Tutorial 1: What is a Node?

A **Node** is just a single program. That's it.

Think of a robot: you might have one program (a node) for the camera, one for the wheels, and one for making sounds. They're all individual programs, and ROS 2 helps them work together.

### 1. Let's Run a Node!
`turtlesim` is the "Hello, World!" of ROS. It's a little turtle simulator. If you don't have it, install it:

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

You might have to run many terminals for this.
Now, let's run the `turtlesim` node:
```bash
# In Terminal 1
source /opt/ros/humble/setup.bash #add this to your bashrc if you can to avoid doing it everytime
ros2 run turtlesim turtlesim_node
```
Poof! A new window with a turtle should appear. That window *is* your node, running and waiting.

### 2. See Your Running Node
Leave that first terminal running. Open a **new terminal** (Terminal 2). You have to `source` in *every... single... new... terminal*. (Don't worry, We ll add that to bashrc.)

```bash
# In Terminal 2
source /opt/ros/humble/setup.bash
```

Now, let's ask ROS 2 what's running:
```bash
ros2 node list
```
It'll spit back `/turtlesim`. That's the name of the node you just launched. Pretty neat, right?

### 3. Let's Run a *Second* Node
Let's run one that can control the turtle.

Open a **new terminal** (Terminal 3).
```bash
# In Terminal 3
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtle_teleop_key
```
This terminal will just sit there, waiting for your input. This is also a node!

### 4. Check Your Node List Again
Go back to **Terminal 2** and run the `list` command again:
```bash
ros2 node list
```
Now you'll see two nodes:
* `/turtlesim`
* `/teleop_turtle`

You have two separate programs running. But... how are they connected?

**Go try it!** Click on Terminal 3 (the `teleop_key` one) and use your arrow keys. The turtle in the other window moves! 🐢💨 They're communicating behind the scenes using **Topics**.

---

## Tutorial 2: What's the Topic?

**Topics** are how nodes talk. It's like a public radio station.

* A node can **publish** (broadcast) messages on a topic.
* Any other node can **subscribe** (listen) to that topic.

The nodes don't know or care who is on the other end. They just shout into the void (publish) or listen to the void (subscribe).

### 1. Find the Topics
With your turtle stuff still running, go to **Terminal 2** and run:
```bash
ros2 topic list
```
You'll see a few, but the important ones are:
* `/turtle1/cmd_vel` (This is the "command velocity" topic. The keyboard node publishes here.)
* `/turtle1/pose` (The turtle sim publishes its X/Y position here.)

### 2. Eavesdrop with `ros2 topic echo`
`echo` is your high-tech eavesdropping tool. Let's listen in on the turtle's position.

In **Terminal 2**, run:
```bash
ros2 topic echo /turtle1/pose
```
It'll just sit there. Now go to **Terminal 3** and use your arrow keys. Look back at Terminal 2, you're seeing a live stream of the turtle's position data! Press `Ctrl+C` in Terminal 2 to stop.

### 3. Control the Turtle with `ros2 topic pub`
`pub` (publish) lets *you* be the one giving commands. Let's tell the turtle to move without using the keyboard.

This command is a good, but it's just sending one single message.
Run this in **Terminal 2**:
```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
Your turtle should immediately move forward and spin. You just published your first message!

---

## Tutorial 3: What is a Service?

**Services** are different from topics. If a topic is a radio broadcast (one-way, continuous), a service is a phone call.

* You **call** a service with a **request**.
* You wait.
* You get a **response** back.

It's a two-way transaction.

### 1. See What Services are Available
You guessed it. In **Terminal 2**, run:
```bash
ros2 service list
```
You'll see a bunch. Let's look at `/spawn`. This service, as you might guess, lets us spawn a new turtle!

### 2. Call the Spawn Service
We need to tell the service *where* to spawn the turtle and what its name should be.

Run this in **Terminal 2**:
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: 'turtle2'}"
```
A new turtle (`turtle2`) appears in your simulator. And your terminal shows you the *response* from the service: `{name: 'turtle2'}`.

The `/spawn` service is *turt-ally* at your service. (Sorry, I'm not sorry.)

---

## Tutorial 4: ROS Doctor

Things will break. Nodes won't talk. Topics will be missing. It happens. Before you panic, call the doctor. Doctor who?

`ros2 doctor` is a diagnostic tool that checks your setup for common problems.

### 1. Get Your Check-up
In **Terminal 2**, just run:
```bash
ros2 doctor
```
It'll run a bunch of checks on your network, your environment, and everything else.

### 2. Read the Diagnosis
If all is well, it'll tell you. But if there's a problem (like your firewall is blocking things, or two nodes have the same name), `doctor` will give you a warning. It's the first place you should look when things dont work.

---

And... that's it! You now know about **Nodes**, **Topics**, and **Services**, which are the main of ROS 2.
