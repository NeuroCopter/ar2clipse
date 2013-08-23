# ar2clipse #

Utilities to convert an ardupilot sketch to an eclipse project


## tested on ##

Ubuntu  13.04 Eclipse 4.2 CDT + AVR Plugin, apm2

Mac OSX 10.7  Eclipse 4.2 CDT + AVR Plugin, apm2

## why ##

* instead of seperate header and source files, arduino's very own pde-system uses a single file and allows the use of functions, that haven't been declared prior.
* Omitting required forward-declarations produces semantically invalid c(++) code
* upon building a sketch, it's main .pde file together with all other .pde-files and forward-declarations of all functions is merged into one huge .cpp-file.
* arduino has a unique way of handling includes: "#include <${LIB}.h>" actually refers to "libraries/${LIB}/${LIB}.h". Therefore "-Ilibraries/${LIB}" is passed to the compiler.

→ because of these facts, treating an arduino sketch as a c(++)-project fails 


## what the script does (using the example of ArduCopter) ##

* build the ArduCopter-project using the ardupilot-makefile to obtain the large ArduCopter.cpp (because we didn't want to reproduce the file-merging done by the makefile)
* symlink libraries-folder into the eclipse project
* symlink all *.pde as *.pde.hpp into the ArduCopter folder (in the eclipse project)
* replace the code from the .pde files by corresponding includes of the *.pde.hpp files in the ArduCopter.cpp and copy it in the ArduCopter folder in eclipse
* add all used library-folders to the include-paths
* exclude unused libraries and /example/ -subfolders


# initial setup #

### install required software ###

* [eclipse cdt](http://www.eclipse.org/downloads/) (tested only with 4.2) 
* [eclipse avr plugin](http://avr-eclipse.sourceforge.net/updatesite)
* avr-toolchain

### create eclipse project ###

* New > C++ Project > AVR Cross Target Application > Empty Project [AVR-GCC Toolchain]
* MCU Type: ATmega2560; MCU Frquency: 16000000
* [Finish]

### configure project ###

__C/C++ Build__
> Settings [Tool Settings]

>> AVR Compiler
>>> Symbols
>>> * define the symbols according to the sketch you want to build, e.g.  

>>>> SKETCH="ArduCopter"  
>>>> FRAME_CONFIG=QUAD_FRAME  
>>>> CONFIG_HAL_BOARD=HAL_BOARD_APM2  

__If you use eclipse just for editing the code (and use the ardupilot-makefile for building), the following steps (setting compiler flags) can be skipped.__
_(although the flags are pretty much the same as in the makefile, the sizes of the generated binaries differ.)_

>> AVR Assembler
>>> Debugging
>>> * -g2
>>> * operating system default

>> AVR Compiler
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

>> AVR C++ Compiler ( _same settings as for AVR Compiler with following additions_ )  
>>> Language Standard
>>> * -fno-exceptions

>>> Miscellaneous > Other flags
>>> * -Woverloaded-virtual -Wsign-promo

>> AVR C++ Linker  
>> Command:   
>> * avr-gcc (replace avr-g++) 

>>> General > Other Arguments
>>> * -mcall-prologues -Os -Wl,--gc-sections -Wl,-m,avr6 -Wl,--relax

>>> Libraries > -l
>>> * m


# how to use it #

### sym-link ArduPilotMega files into the local project ###

------------
_These steps have to be executed whenever one of the following occurs:_  
* ArduPilotMega project is updated from external source (git)
* using a priorly unused library
* if .pde files are altered in a way, that results in a change to the auto-generated big .cpp file (e.g. new functions are added)   
   
------------   
   
* close the eclipse project (since the project's config file is edited by the script)
* run the script

> * e.g. python main_cmd_line.py ~/workspace ArduCopter ~/git/ardupilot ArduCopter -a=apm2-quad  
> * see python main_cmd_line.py --help for a detailed parameter description

* reopen the project, refresh & rebuild the index


### how we use it ###

We've created an eclipse workspace that contains 3 projects:

1. ar2clipse (synced with github using the egit plugin)
2. ArduPilotMega (synced with github using the egit plugin)
3. the local c++ project described above

* All work is done on the local project. because of the symlinks, all changes are actually performed on the ArduPilotMega files. (except for the generated ArduCopter.cpp file)
* Therefore the changes can be commited right away via the ArduPilotMega project in eclipse (or through the git tool of your choice).
* whenever we update the ArduPilotMega project (pull / fetch), we refresh the local project by running the script as described above.

 
