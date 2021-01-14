# CubeSat-PCBSat-Command-and-Communication
- This software is being written to allow for a CubeSat to relay commands from a ground station to a swarm of femtosatellites.
- The software for this project was written by adapting code examples written by TI and made available on the SimpleLink platform.
- This code enables a CubeSat to be able to relay commands from a ground station to a femtosatellite, as well as relaying data from the femtosatellite to the ground station.

# Hardware
The targeted hardware platform for this code is the CC1310 SoC by TI. The CC1310 Launchpad is the hardware development kit manufactured by TI.
This code was tested on these HDKs. Further advice on programming a custom design with a CC1310 SoC should be sought through TI's various resources.
All information and resources relating to the CC1310 can be found on https://www.ti.com/product/CC1310/technicaldocuments.

# Instrucitons for use
The IDE used to develop the software for this project was Code Composer Studio by TI.

If anyone viewing this repo wishes to run these examples on CC1310 Launchpads, they must create three seperate CCS projects:
- They should both contain all of the files in this repo with the exception of three files.
- The board the user wishes to act as the ground station will need to use the "GroundStation_nortos.c" file.
- The board the user wishes to act as the CubeSat will need to use the "CubeSat_nortos.c" file.
- The board the user wish to act as the femtosatellite will need to use the "Femtosatellite_BB_nortos.c" file.

It is recommended that the user imports the pre-made EasyLink Echo CCS projects into their workspace. Once this is done, they can copy the replace the primary .c files with the aforementioned files in this repo.

The user must also change the addresses in the easylink_config.h files for each device since address filtering is used by default. 

The current version of this software has a Tx output power of 8dBm, but this can easily be changed using SmartRF Studio or by adapting the "smartrf_settings.c" file.

The examples this code was based on, along with any further help with the SimpleLink platform, can be found on http://dev.ti.com/tirex/explore/node?node=AMoBnKEgI1TSJ3K1G3NR6w__eCfARaV__LATEST.

# Board Output
- Any UART terminal app can be used, the student used PuTTY.
- They will print the appropriate message on screen as well as blinking different LEDs upon reception/trasmission.
- Since there are so many possible outcomes, it is recommended that the user uses the UART display option, since what is going on is much clearer.
