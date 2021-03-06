# Kinova-Bilaterals

## Robots Supported

* Kinova Gen3 Ultra 7DOF
  * API - 2.09 +

## Unilateral

Will test the robot in torque mode, with Compensation and PD control.

The program will initially connect to the robot.
Then, clear faults on the robot and request the robot to home.
The robot will then go into low-level torque mode
with a PD controller holding the home position in 1kHz torque mode.

With compensation: \f$u = (u+C(p,\dot{p}) +G(p))\f$, where, 
\f$p\f$ is position and \f$\dot{p}\f$ is velocity of the robot joints. (N.b. \f are there for doxygen rendering of math mode, but breaks markdown rendering mathmode).

There is a topic that can set the demand position and velocity, overwriting the starting home position.
This allows for position control, for velocity control you need to feed in current position as well.

## Bilateral

Will connect 2 robots together with positional force feedback.

The program will initially connect to the robots.
Then, clear faults on the robots and request the robots to home.
The robot will then go into low-level torque mode
with a PD controller holding the robots in the same position and velocity in 1kHz joint torque mode.

With compensation: \f$u = (u+C(p,\dot{p}) +G(p))\f$, where, 
\f$p\f$ is position and \f$\dot{p}\f$ is velocity of the robot joints.


## Shared Features

### Pause
Each robot has a hold position controller, triggered by a ROS parameter.

### PD Tuning
Each robot can be live tuned for PD k parameters via ROS parameters

### Compensation 
Both gravity and coriolis compensaiton can be turned off and on by ROS parameter for each robot.


## Build

### Dependencies

* ROS2 foxy - for logging
* conan for kinova
* build-essentials

### Optional dependencies

* Doxygen
* LaTeX - for lateX doxygen docs

### Standard build

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr
make -j$(nproc)
```

### Build AppImage

```bash
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr

# build project and install files into AppDir

make -j$(nproc)

make install DESTDIR=AppDir

linuxdeploy-x86_64.AppImage --appdir AppDir -o appimage -d ../desktop/unilateral_mock.desktop -i ../desktop/logo.png -l /opt/ros/foxy/lib/librmw_fastrtps_cpp.so -l /opt/ros/foxy/lib/librmw_dds_common__rosidl_typesupport_fastrtps_cpp.so -l /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so -l /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so -l /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so 
```

```
linuxdeploy-x86_64.AppImage --appdir AppDir -o appimage -d ../desktop/unilateral.desktop -i ../desktop/logo.png -l /opt/ros/foxy/lib/librmw_fastrtps_cpp.so -l /opt/ros/foxy/lib/librmw_dds_common__rosidl_typesupport_fastrtps_cpp.so -l /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so -l /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so -l /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_fastrtps_cpp.so 
```
This will enable the 

## Additional Repo Features

* devcontainer support
* gitpod support
* vs code setup support

## Tests

### Mocking 

There are 2 mocking classes which mocks the hardware interfaces, to allow for full system testing without hardware.

### Unit tests

* PD-Test - Test for the PD controller
* KinovaTest - Tests the robotMath function 