# Raspberry Pico RP2040 Robotic Project
***
## Software

<ul>
  <li> <a href="https://github.com/raspberrypi/pico-sdk">Raspberry Pi Pico SDK</a> (latest stable release)</li>
  <li> <a href="https://github.com/FreeRTOS/FreeRTOS-LTS/tree/202406-LTS">FreeRTOS-LTS</a> (v202406-LTS) </li>
  <li> <a href="https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git">micro-ROS module for Raspberry Pi Pico SDK</a> (coming soon!)</li>
</ul> 

## Getting Started
1. `git clone` this repository
```bash
git clone --recurse-submodules https://github.com/tutla53/robotic-arm-rp2040.git
```
2. Clone the SDK as a submodule called `pico-sdk`
3. Setup a `CMakeLists.txt` in the project directory like:

```cmake
cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
set(PICO_SOURCE ../pico-sdk)
include(${PICO_SOURCE}/pico_sdk_init.cmake)

project(my_project)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project
```
4. Go to the project directory, create the build folder, and compile the software:
```bash
cd {project_directory}
mkdir build
cd build
cmake ..
make -j4
```
