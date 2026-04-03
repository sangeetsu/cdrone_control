# Documentation: Motive (MoCap) to ROS 2 Setup

## 1. Setup Server on Motive (MoCap software)
1. Go to **Edit** -> **Streaming**.
2. Make it look like the image below:
   See inside Google Drive link: [Drive](https://docs.google.com/document/d/17S1W8AmRlu-KmQCbGPHSAudPQJPVxJ46zFvBXcdX4JA/edit?tab=t.0)
3. Scroll to the bottom and select **"enable"** under VRPN.
4. Press the **red button** to begin streaming.

## 2. Making a RigidBody
1. Make sure there are IR markers in the drone studio (and the mocap cameras are on!).
2. You should see little dots on the Motive software. Find them and select them all (click away from them, then drag over every dot you want to make into one rigid body).
3. A drop-down should pop-up. Select **"create RigidBody"**.

### ⚠️ Naming Conventions
Inside Motive, you **must** name all important objects as `RigidBody#` with the `#` being some number (ex: `RigidBody1` or `RigidBody2`). 

> **Important:** If you don’t do this, it won’t be detected in ROS. Don’t ask me why. Always name them `RigidBody#` or else ROS won’t detect it.

## 3. Run ROS Package (`vrpn_mocap`)
**Repository:** [alvinsunyixiao/vrpn_mocap](https://github.com/alvinsunyixiao/vrpn_mocap)

### Prerequisites
* Make sure you are on the same network as the Motive server.
* Make sure ROS and the package are installed. 
* Make sure the workspace is created (it should already be created).

### Running the Client
Inside your workspace, in **every** session (terminal), you must source the environment:
```bash
source install/setup.bash
```

Then run the package with this command: 
```bash
ros2 launch vrpn_mocap client.launch.yaml server:=192.168.0.217 port:=3883
```