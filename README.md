# multiplayar-core
[![Build Status](https://travis-ci.org/jacobkahn/multiplayar-core.png?branch=master)](https://travis-ci.org/jacobkahn/multiplayar-core)

[MultiplayAR](http://multiplayar.me/) is a framework for building multi-client augmented reality experiences. This repository houses core infrastructure for a `multiplayar` deployment. This includes:
- Data transmission layers
- State management for clients and AR objects
- Image processing and transformation computation

Client code for iOS [can be found here](https://github.com/jacobkahn/multiplayar).

## Requirements

- A C++ compiler with good C++11 support (i.e. g++ >= 4.8)
- [cmake](https://cmake.org/) - 2.8 or later
- [Boost](http://www.boost.org/) - 1.60.0 or later
- [OpenCV](https://opencv.org/opencv-3-4.html) - 3.0 or later
- [Crow](https://github.com/ipkn/crow) - included with the project in `include/deps`

## Compiling and Running the Project

Build the project with `CMake` and `make`.

To the build project:
```
mkdir build bin
cd build
cmake ..
make
```

To run the project:
```
cd bin
./multiplayar-core
```