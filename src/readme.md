# Design and Reproduce
Feel free to contact me via email: `lujhcoconut@foxmail.com` .
 This project is based on `banshee` open-source project. The link to the open-source Banshee project is `https://github.com/yxymit/banshee` .

## Modified Directory
```shell
|-src
    |-page_placement.cpp
    |-page_placement.h
    |-mc.cpp
    |-mc.h
    |-init.cpp
    |-memory_hierarchy.h
    |-memory_hierarchy.cpp
    |-ddr_mem.cpp
    |-ddr_mem.h
    |-...
    |-...
```


### SDCache 
The design of SDCache involves modifications to the following files.
```shell
|-src
    |-page_placement.cpp
    |-page_placement.h
    |-mc.cpp
    |-mc.h
```
* `page_placement.h` and `page_placement.cpp` are the main components of SDCache's implementation of the PRR policy.
* `mc.cpp` and `mc.h` are the core components of SDCache's fundamental data structures and program execution.

### Trimma (PACT'24)
The design of Trimma involves modifications to the following files.
```shell
|-src
    |-mc.cpp
    |-mc.h
```
* `mc.cpp` and `mc.h` are the core components of Trimma's fundamental data structures and program execution.