# Autonomous Flight Module ROS
[ROS](http://www.ros.org/) package for autonomous flying of a drone based on local surface estimation.

> **Author**: Marin, Borja </br>
> **Affiliation**: Heriot-Watt University </br>

## Description
This development includes a set of methods for autonomous surveying of walls using drones. Testing has been carried out in an indoor environment and, therefore, 3D position and orientation of the drone is captured using VICON cameras. Information about the drone's local environment is acquired using a set of three ranging sensors placed in the nose of the drone.

![](img/SUI_Endurance_Complete.png)

### Steps followed for autonomy
1. **Retrieve UAV's global position & orientation**: we use [vicon bridge](https://github.com/ethz-asl/vicon_bridge) to capture this information.
2. **Sensor range conversion to 3D point**: transform distances to 3D points using a transformation matrices ([tf2](https://wiki.ros.org/tf2)). These points will be refered to the local reference frame fixed to the drone's body.
3. **Local surface estimation**: a simple plane-fitting function characterises a plane by its normal vector and distance to a reference frame (local frame of the drone).
4. **3D $\rightarrow$ 2D projection**: using the constraint that the drone will be perpendicularly facing the
wall and knowing the parameters of the plane fitted to the local surface, we obtain the 2D projection of the drone’s 3D position.
5. **Control signals calculation**: a set of three PID controllers generate the required velocities to achieve a smooth autonomous flight.
6. **Publish target state**: the target velocities are published to the ROS network.

### ROS Nodes
- **/mavros**: acts as the interface between the aircraft and the control command.
- **/vicon**: publishes the drone’s global position and orientation. Since the drone cannot operate autonomously unless it has data for its global position, Vicon’s data is republished to mavros’ 
 `vision_pose/pose` and `/fake_gps/mocap/pose` topics through a parrot-like node (`vicon_listener`).
- **/camera**: handles anything related to capturing images. RGB images are published on `/color/image_raw/compressed` topic which can be subscribed by an external application to run image processing pipelines.
- **/follow_path**: his node gathers all the data from the ranging sensors and drone state by subscribing to the respective topics, does the necessary projections and transformations from 3D space to 2D, executes the control sequences for every degree of freedom of the drone and generates the new velocity targets in order to track the trajectory provided by the user. These commands are published to mavros’ `/setpoint_velocity/cmd_vel` and sent to the onboard control unit.

![](img/rqt_graph.jpeg)

## Key features of package
- 2D position control of drone
- Plane fitting to local surface
- 2D path reconstruction in 3D space
- Linear $(v_x,v_y,v_z)$ and rotational $(\theta_z)$ velocities generation

## Hardware used for testing
- [VICON Vero](https://www.vicon.com/hardware/cameras/vero/) positioning cameras
- [Nvidia Jetson Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) Developer Kit
- [Pixhawk Cube Black](https://ardupilot.org/copter/docs/common-thecube-overview.html)
- [Teraranger Evo 3m](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-3m/)
- [SUI Endurance](https://www.hiteccs.com/drones/products) drone
