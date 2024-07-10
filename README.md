# Raspberry Pico RP2040 Robotic Project
***
## Software

<ul>
  <li> <a href="https://github.com/raspberrypi/pico-sdk">Raspberry Pi Pico SDK</a> (latest stable release)</li>
  <li> <a href="https://github.com/FreeRTOS/FreeRTOS-LTS/tree/202210-LTS">FreeRTOS-LTS</a> (v202210-LTS) </li>
  <li> <a href="https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git">micro-ROS module for Raspberry Pi Pico SDK</a> (coming soon!)</li>
</ul> 

## Getting Started
1. `git clone` this repository
    ```bash
    git clone https://github.com/tutla53/rp2040.git
    ```
2. Move to the SDK and FreeRTOS-LTS as a submodule `pico-sdk` and `FreeRTOS-LTS`
3. Initialize and update the submodule
    ```bash
    git submodule update --init
    ```
    | :warning: WARNING          |
    |:---------------------------|
    |Don't recurse git submodules|

4. Setup a `CMakeLists.txt` in the project directory like:

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
5. Go to the project directory, create the build folder, and compile the software:
    ```bash
    cd {project_directory}
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
