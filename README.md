# CubeSat-PCBSat-Command-and-Communication
- This software is being written to allow for a CubeSat to relay commands from a ground station to a swarm of femtosatellites.
- The software for this project was written by adapting code examples written by TI and made available on the SimpleLink platform.

# Hardware
The targeted hardware platform for this code is the CC1310 SoC by TI. The CC1310 Launchpad is the hardware development kit manufactured by TI.
This code was tested on these HDKs. Further advice on programming a custom design with a CC1310 SoC should be sought through TI's various resources.
All information and resources relating to the CC1310 can be found on https://www.ti.com/product/CC1310/technicaldocuments.

# Instrucitons for use
The IDE used to develop the software for this project was Code Composer Studio by TI.

If anyone viewing this repo wishes to run these examples on CC1310 Launchpads, they must create two seperate CCS projects:
- They should both contain all of the files in this repo with the exception of two files.
- The board the user wishes to act as the CubeSat will need to use the "rfEchoTx.c" file.
- The board the user wish to act as the femtosatellite will need to use the "rfEchoRx.c" file.

The current version of this software has a Tx output power of 8dBm, but this can easily be changed using SmartRF Studio or by adapting the "smartrf_settings.c" file.

The examples this code was based on, along with any further help with the SimpleLink platform, can be found on http://dev.ti.com/tirex/explore/node?node=AMoBnKEgI1TSJ3K1G3NR6w__eCfARaV__LATEST.

# Board Output
- The CubeSat board will blink green upon successful transmission and reception of a packet.
- The femtosatellite board will blink red if it receives a packet that is intended for a different femtosatellite.
- The femtosatellite board will blink red and green if it receives a packet that has an address which matches the address of the femtosatellite.
