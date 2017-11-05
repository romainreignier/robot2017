# robot2017

Software for the robot 2017.

- `firmware`: firmware of the NucleoL476 board, based on ChibiOS RTOS.
- `ros_ws`: workspace ROS to be run on the embedded computer.

## System dependencies

Ubuntu 16.04 with `ros-kinetic-dektop-full` is needed.

## Build

Make sure you initialize and update the submodules in order to build.

    $ git clone https://github.com/romainreignier/robot2017.git
    $ cd robot2017
    $ git submodule init
    $ git submodule update

## Firmware

Install a compiler suite (but a bit old):

    $ sudo apt install binutils-arm-none-eabi gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib

For Openocd, build from sources to get the support for the NucleoL476:

    $ sudo apt install git autoconf libtool make pkg-config libusb-1.0-0 libusb-1.0-0-dev
    $ git clone git://git.code.sf.net/p/openocd/code openocd-code --depth=1
    $ cd openocd-code/
    $ ./bootstrap 
    $ ./configure 
    $ make
    $ sudo make install

Add user to `dialout` group (login or reboot needed)

    $ sudo adduser $USER dialout

### Rosserial
To generate the rosserial headers:

    $ cd ros_ws
    $ source devel/setup.bash
    $ cd ../firmware/common
    $ rm -r ros_lib
    $ rosrun rosserial_chibios make_libraries.py .
