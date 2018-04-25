Arduino CC2500 Ping-Pong
========================

This is a basic implementation of a Ping-Pong mechanism between two CC2500
devices. It is by no means bulletproof, but meant to illustrate how
to get the TI CC2500 module up and running.

It is tested on Arduino Leonardo, but is should work on any AVR
Arduino (however, it may be necessary to change some of the pins, see
below).

The default frequency is 2.408 GHz. This can be changed using the
`FREQ` definition in the top of `CC2500.ino` (see also practical
considerations below).

The two CC2500 devices are connected to the same Arduino board and
shares the same SPI bus (with separate chip select and GDO0 pins).
Each CC2500 device is controlled by separate tasks which are executed
concurrently using the krnl (see below). However, it is
straightforward to use the code in a setup with two Arduinos each
controlling a single CC2500 device. In order to do so, simply comment
out the krnl specific code (mutexes, `k_crt_task(...)` calls, etc.)
and call the `task(...)` function as the last step of `setup()`.

## Pin configurations
The SCLK, MOSI, MISO pins of the CC2500 devices must be connected to
the corresponding pins on the Arduino board. Both CS2500 devices share
the same SCLK, MOSI, and MISO pins on the Arduino board.

The chip select (CS) pin of each of the CC2500 must be connected to
distinct pins on the Arduino board. In the default configuration, pin
12 is used for CS of device 1, while pin 11 is used for CS of device
2.

Similarly, the GDO0 pins of the CS2500 devices must also be connected
to the Arduino. It is important that these pins are connected to pins
that are capable of external interrupts. By default, GDO0 of device 1
is assumed to be connected to pin 7, and GDO0 of device 2 to pin 3.

## Practical considerations
The CC2500 uses the 2400-2483.5 MHz ISM band with carrier sense, and
the Ping-Pong may not work as intended if there are many interferes.
No retransmissions (or even detection of packets not being transmitted
due to the carrier sense mechanism) is implemented.
Therefore, it may be necessary to change the frequency used by the
CC2500 within this band to a frequency that has less interference. The
specific frequency used by the CC2500 can be configured by the `FREQ`
definition in the top of the `CC2500.ino` file.

## KRNL
The project requires [krnl](https://github.com/jdn-aau/krnl) which is
a simple and lightweight kernel for the Arduino platform. Simply copy
the `krnl.[ch]` files to a `krnl` folder in the Arduino library
directory. _This project has been tested with commit 
`99d0132490fca658780fbcb5348a30e189e5407f`._


