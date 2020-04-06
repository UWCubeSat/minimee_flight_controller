# NanoLab
Code for NanoLab flight controller

## Setting up the interactive simulator

First, compile and install the latest version of [simavr](https://github.com/buserror/simavr) with development headers. On debian, `apt install simavr-dev`. Elsewhere:

```sh
git clone https://github.com/buserror/simavr
cd simavr
make
sudo make install
```

Then, download and install the [Arduino CLI](https://arduino.github.io/arduino-cli/installation/), putting it in your `$PATH`. Install necessary libraries under the Arduino CLI (this may not be necessary if you've already used the Arduino IDE GUI):

```sh
arduino-cli core install arduino:avr
arduino-cli lib install SD
```

Now, set up an empty SD card image.

```sh
truncate -s 1G something.img
fdisk something.img 
# Create new partition: type n, then use defaults
# Write partitions to disk: type w
losetup -fP --show something.img
mkfs.fat /dev/loop0p1 # Make sure it's the right loop device!
losetup -d /dev/loop0
```

The contents of the SD card can be viewed at any time in the future:

```sh
losetup -fP --show something.img
mount /dev/loop0p1 /mnt
less /mnt/LOG.TXT
umount /mnt
losetup -d /dev/loop0
```

Finally, you can build the nanolab interactive simulator:

`make`

This compiles the Arduino sketch and the simulator. Finally, run:

`tests/interactive blue_origin_fc/blue_origin_fc.arduino.avr.uno.elf ./path/to/sd.img`

### Usage

Press Ctrl-C to run a control command.

 + `voltage=3.8`: Set the voltage sensor, in volts.
 + `current=0.025`: Set the current sensor, in amps.
 + `temperature=27`: Set the temperature sensor, in temperature.
 + `quit`: Stop the simulation.
 + Anything else: Send over serial.
 
When setting a sensor input, specify the logical value. The simulator will
automatically convert these values to the voltages that the circuit will
actually deliver to the Arduino.
