This is a collection of programs and Arduino sketches made for research for LoRa localization with Heltec WiFi Lora V3 devices.

## Installation
* Start by cloning this repository.
* Install the Arduino IDE and the Heltec libraries. There is some sort of version incompatibility going on. If the most recent versions don't work for unknown reasons, try the esp32 v3.1.3 in the board manager and Heltec ESP32 Dev-Boards v2.1.2 in the library manager.
* A recent version of Python 3 and some common modules like numpy and pandas are used in some of the python programs
* Copy secrets_example.h to a new file secrets.h and replace the values with the correct wifi ssid, password, and server ip. If you don't change the port in the python file, the DEST_PORT field can remain the same. (This is necessary for the programs that connect back to a computer over wifi)

## Usage
* For Arduino sketches, connect the device, select board and port, then press upload. Read the sketches to figure out how to interact with them (since the input is just a button, the actions are almost certain to be short press or long press)
* For collection-server, the command to run is `python3 server.py`. There are no command arguments. Shutdown the server with control-c.
* For experiment-runner, there are command line arguments. Check the help menu with `python3 runner.py -h` and follow the instructions.

## Protocol details
* Currently the protocols start with a 4-byte magic number, usually a CRC32 to make them arbitrary.
* The protocols use network/big endian in the header.
* Then 8 bytes for 2 bytes worth of flags and then 6 bytes of the chip id, which is the EfuseMAC
* In protocols 0xba41dba8, 0x61462cdf, the first bit specifies whether the data section is in little-endian (1) or big-endian (0).