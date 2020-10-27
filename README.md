# CubeSat-PCBSat-Command-and-Communication
- This software is being written to allow for a CubeSat to relay commands from a ground station to a swarm of femtosatellites.
- The software for this project was written by adapting code examples written by TI and made available on the SimpleLink platform.
If anyone viewing this repo wishes to run these examples on CC1310 Launchpads, they must create two seperate CCS projects:
- They will both contain all of the files in this repo with the exception of two files.
- The board the user wishes to act as the CubeSat will need to use the "rfEchoTx.c" file.
- The board the user wish to act as the femtosatellite will need to use the "rfEchoRx.c" file.
