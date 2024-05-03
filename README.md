# Autonomous Flight Module ROS
[ROS](http://www.ros.org/) package for autonomous flying of a drone based on local surface estimation.

> **Author**: Marin, Borja </br>
> **Affiliation**: Heriot-Watt University </br>

## Description
This development includes a set of methods for autonomous surveying of walls using drones. Testing has been carried out in an indoor environment and, therefore, 3D position and orientation of the drone is captured using VICON cameras. Information about the drone's local environment is acquired using a set of three ranging sensors placed in the nose of the drone.

![](img/SUI_Endurance_Complete.png)

### Steps followed for autonomy
1. Retrieve UAV's global position & orientation: we use [vicon bridge](https://github.com/ethz-asl/vicon_bridge) to capture this information.
2. Sensor range conversion to 3D point: transform distances to 3D points using a transformation matrices ([tf2](https://wiki.ros.org/tf2)). These points will be refered to the local reference frame of the drone.

## Key features of package
- 2D path reconstruction in 3D space
- Plane fitting to local surface
- Linear $(v_x,v_y,v_z)$ and rotational $(\theta_z)$ velocities generation
- 3D position control of drone

## Hardware used for testing
- [VICON Vero](https://www.vicon.com/hardware/cameras/vero/) positioning cameras
- [Nvidia Jetson Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit) Developer Kit
- [Pixhawk Cube Black](https://ardupilot.org/copter/docs/common-thecube-overview.html)
- [Teraranger Evo 3m](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-evo-3m/)
- [SUI Endurance](https://www.hiteccs.com/drones/products) drone
