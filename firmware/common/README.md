# Patch ChibiOS-Contrib

ChibiOS-Contrid does not support yet the L4xx family.

You need to add a file:

    $ cp -r STM32L4xx  ChibiOS-Contrib/os/hal/ports/STM32/
