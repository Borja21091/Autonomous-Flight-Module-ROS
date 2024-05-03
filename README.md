# Autonomous Flight Module ROS
[ROS](http://www.ros.org/) package for autonomous flying of a drone based on local surface estimation.

## Description
This development includes a set of methods for autonomous surveying of walls using drones. Testing has been carried out in an indoor environment and, therefore, 3D position and orientation of the drone is captured using VICON cameras. Information about the drone's local environment is acquired using a set of three ranging sensors placed in the nose of the drone.

[!](img/SUI_Endurance_Complete.png)

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
