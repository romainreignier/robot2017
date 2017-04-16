# robot2017

Software for the robot 2017.

- `firmware`: firmware of the NucleoL476 board, based on ChibiOS.
- `ros_ws`: workspace ROS to be run in the embedded computer.

## Build

Make sure you initialize and update the submodules in order to build.

    $ git clone https://github.com/romainreignier/robot2017.git
    $ cd robot2017
    $ git submodule init
    $ git submodule update

### Protobuf msgs

To generate protobuf files, make sure you installed the following packages: `protobuf-compiler` `python-protobuf`.

Then go to the `msg` directory to generate the messages for the ROS nodes:

    $ cd msg
    $ make

Go to `firmware/common/nanopb/generator/proto` and run `make`.

    $ cd firmware/common/nanopb/generator/proto
    $ make

## Firmware

Install a compiler suite `binutils-arm-none-eabi` `gcc-arm-none-eabi` `libnewlib-arm-none-eabi` `libstdc++-arm-none-eabi-newlib`  and `openocd`.

Add user to `dialout` group (login or reboot needed)

    $ sudo adduser $USER dialout

### Rosserial
To generate the rosserial headers:

    $ cd ros_ws
    $ source devel/setup.bash
    $ cd ../firmware/common
    $ rosrun rosserial_client make_libraries .
