* Currently the protocols start with a 4 byte magic number, usually a CRC32 to make them arbitrary.

* The protocols use network/big endian.

* I used to have the next 4 bytes be an extra number for info like version and such.
I am removing that since we don't benefit much from semantic versioning.

* Then 8 bytes for 2 bytes worth of flags and then 6 bytes of the chip id, which is the EfuseMAC