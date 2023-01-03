# Copyright
2023 Darren Jones (nz.darren.jones@gmail.com)

Based on the work by:

- Simon Inns https://github.com/simoninns/SmallyMouse2
- sekigon-gonnoc https://github.com/sekigon-gonnoc/Pico-PIO-USB

# RP2040 Mouse
This is the software component of my RP2040 based USB to Quadrature mouse adaptor

This is a mouse adapter designed to work with systems that use expensive and hard to find Quadrature mice. It is based around the dual core RP2040 microcontroller and designed to be as small as possible.

**Compatible with**

- Amiga 500
- Amiga 500+
- Amiga 1000
- Amiga 1200
- Amiga 2000
- Amiga 3000
- Amiga 4000
- Atari 520ST
- Atari 1040ST
- Atari TT
- Atari Falcon 030

#### Why did you make it?

Mice for retro computers are getting harder to find and more expensive all the time, there were a few other adapters like this on the market but they all fell short in one way or another. 


#### What makes it special?

My adapter supports a few things others do not
- Easy firmware updates over USB.
- External power for mice that draw more current than the computer the adapter is attached to can supply, this is particularly an issue with wireless mice on Atari STs.
- Status LEDs to let you know if it is working or not.
- Integrated DB9 which is switchable between Atari ST and Amiga mode, this avoids bulky cables.
- Mouse scroll wheel support

# Building

To build this software you will need:
- a working TinyUSB SDK (https://github.com/hathach/tinyusb)
- a working Pico PIO USB SDK (https://github.com/sekigon-gonnoc/Pico-PIO-USB)
- CMake Tool Chain

To build the software clone this repository and then run the following
```
PICO_PIO_USB_DIR=/path/to/Pico-PIO-USB PICO_TINYUSB_PATH=/path/to/tinyusb BOARD=pio_sdk cmake ..
make
```

Once the build completes you will see the following:

```
[100%] Linking CXX executable rp2040_mouse.elf
Memory region         Used Size  Region Size  %age Used
           FLASH:       44524 B         2 MB      2.12%
             RAM:       16552 B       256 KB      6.31%
       SCRATCH_X:          2 KB         4 KB     50.00%
       SCRATCH_Y:          0 GB         4 KB      0.00%
[100%] Built target rp2040_mouse
```

# Installing

Jumper the USB_Boot jumper on the adaptor and plug it into a computer with a USB A to USB A cable, then copy `rp2040_mouse.uf2` built above to the USB drive that the board will expose.
