TODO List:
- [ ] Write to Sectors on SD Card to resolve 2kb SRAM issue, we don't have enough SRAM for a 512B buffer.
- [ ] Implement Self Tests for the MPU6050.
- [ ] Implement Err Sequence for the Logs.
- [ ] Debug Linux FUSE Filesystem Driver for reading SD Card sectors properly.
- [ ] Status LED indication for current State.
- [ ] RF24 Wireless Radio communication, for logging epoch, and sending current state to machine.


This project is an accumulation of code for a on-board Flight Computer for a homemade Rocket.

The current plans are developing a Linux Driver in FUSE to read off the sectors of an SD Card attached to the on-board Arduino.
The reason we write to the Sectors is because Filesystems like FAT and the SD.h library require writting in 512B buffers, but the issue is that we only have 2KBs of SRAM on the Arduino UNO.
To make reading off the SD Card easier and to support mounting it to any Linux system, we're implementing a simple Filesystem Driver. Which is unnecessary, but it's cool and I'm learning stuff.
