# Raspberry Pico RP2040 Robotic Arm Project
***
## Software

<ul>
  <li> <a href="https://github.com/raspberrypi/pico-sdk">Raspberry Pi Pico SDK</a> </li>
  <li> <a href="https://github.com/FreeRTOS/FreeRTOS-LTS.git">FreeRTOS-LTS</a> </li>
  <li> <a href="https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git">micro-ROS module for Raspberry Pi Pico SDK</a> (coming soon!)</li>
</ul> 

## Getting Started
1. `git clone` this repository
```bash
git clone --recurse-submodules https://github.com/tutla53/robotic-arm-rp2040.git
```
2. Set `PICO_SDK_PATH` to the SDK location in your environment
```bash
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```
3. Create the build folder and compile the software:
```bash
mkdir build
cd build
cmake ..
make -j4
```
