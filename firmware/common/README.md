# Patch ChibiOS-Contrib

ChibiOS-Contrid does not support yet the L4xx family.

You need to add a file:

```
$ cp -r STM32L4xx  ChibiOS-Contrib/os/hal/ports/STM32/
```

# Generate `ros_lib`

Build and source the ROS workspace then run in that directory:

```
$ rosrun rosserial_chibios make_libraries.py .
```
