# Hot Metal Brewbot

This project started as an Arduino PID controller for my coffee machine and gradually became a lot more. Read about it [here](http://tomblog.firstsolo.net/index.php/hobbies/pimping-my-coffee-machine/).

Presently there is just a controller PCB and firmware. In time I plan to build a front panel PCB as well. The board is designed to mount neatly in a Gaggia Baby machine but there is scope for other layouts for different machines. The circuit diagram would work with pretty much all machines with some modifications to the firmware.

`hardware/controller` contains EAGLE files for the controller PCB which is an Arduino Nano shield.

`firmware/controller` contains the Arduino source code.

`hardware/controller/DesignNotes.txt` contains a lot of useful information as well as notes about what needs doing next.

The present revision of my controller is 2.0, but I already know of some things I would like to change in 2.1. `DesignNotes.txt` documents these.
