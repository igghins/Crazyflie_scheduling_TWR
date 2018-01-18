# lpsTwrTag.c Modified for Crazyflie 2.0

This is the modified driver of Loco Positioning deck, for different kind of scheduling between Crazyflie and anchors.

Note: Is better to update the macro LOCODECK_NR_OF_ANCHORS to the right number of anchors available.

## Compiling on Ubuntu 16.04 64b

To compiling this drives, the script lpsTwrTag.c must be added manually on the reposity "crazyflie-firmware/src/deck/drivers/src".
To flash the crazyflie 2.0, must be follow the Bitcraze guide in 

https://wiki.bitcraze.io/doc:crazyflie:dev:starting#stm32

 for the Crazyflie 2.0; in particular, opening a terminal on the directory "crazyflie-firmware", must be launched the follow commands:

 ```bash
make clean
make
(sudo) make cload
```

This modified contains three kind of scheduling, numerated from 1 to 3, and the original scheduling:

### Scheduling 1 - Scheduling with exclusion

The scheduling between Crazyflie and anchors is the original one, but this scheduling doesn't permit the communication between the Crazyflie and the two farther anchors, which are escluded.


### Scheduling 2 - Scheduling with frequency (without exclusion)

The scheduling between Crazyflie and anchors happens with specific frequencies, this means that the Crazyflie will communicate with the nearest anchor more frequently than the others, and similarly the other anchors. This frequencies can be modified on the cfclient, because they are parameters.


### Scheduling 3 - Scheduling with frequency and exclusion

The scheduling between Crazyflie and anchors happens with specific frequencies, this means that the Crazyflie will communicate with the nearest anchor more frequently than the others, and similarly the other anchors. Moreover, in this scheduling the Crazyflie doesn't communicate with the two farther anchors.


#### Note

To avoid errors on the link Crazyflie-cfclient, the scheduling happens alternating between the new scheduling proposed and the original one, so that every anchor has all the right information when the Crazyflie moves to the farther anchors.








