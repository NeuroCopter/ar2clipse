ar2clipse
=========

utilities to convert an ardupilot sketch to an eclipse project

tested with
-----------
Ubuntu 13.04 Eclipse 4.2 CDT + AVR Plugin, amp2


howto
=====

install required software
-------------------------
* eclipse cdt (only tested with 4.2) http://www.eclipse.org/downloads/
* avr plugin http://avr-eclipse.sourceforge.net/updatesite
* avr-toolchain

create eclipse project
----------------------
* New > C++ Project > AVR Cross Target Application > Empty Project [AVR-GCC Toolchain]
* MCU Type: ATmega2560; MCU Frquency: 16000000
* [Finish]


configure project
------------------
C/C++ Build
> Setting [Tool Settings]
>> AVR Assembler
>>> Debugging
>>> * -g2
>>> * os default

>> AVR Compiler
>>> Symbols
>>> * e.g. SKETCH="ArduCopter", FRAME_CONFIG=QUAD_FRAME, CONFIG_HAL_BOARD=HAL_BOARD_APM2

>>> Warnings:
>>> * -Wall

>>> Debugging
>>> * -g2
>>> * os default

>>> Optimization
>>> * -Os

>>> Miscellaneous > Other flags
>>> * -Wformat -Wshadow  -Wpointer-arith -Wcast-align -Wwrite-strings -Wformat=2
>>> * -ffunction-sections -fdata-sections -fsigned-char
>>> * -mcall-prologues

>> AVR C++ Compiler

>> **like AVR Compiler with following additions**
>>> Language Standard
>>> * -fno-exceptions

>>> Miscellaneous > Other flags
>>> * -Woverloaded-virtual -Wsign-promo

>> AVR C++ Linker
>> Command: avr-gcc (instead of avr-g++)
>>> General > Other Arguments
>>> * -mcall-prologues -Os -Wl,--gc-sections -Wl,-m,avr6 -Wl,--relax

>>> Libraries > -l
>>> * m

correct some includes in AP_HAL_AVR
-----------------------------------
https://github.com/NeuroCopter/ardupilot/commit/a97fae179fa8b3482bf302a820d29d9ebfb8e4df

copy / link files to eclipse project
------------------------------------
* close the eclipse project / eclipse (since the project's config file is edited by the skript)
* run the skript

> * e.g. python main_cmd_line.py ~/workspace ArduCopter ~/git/ardupilot ArduCopter -a=apm2-quad
