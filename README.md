# multiplayar-core

[MultiplayAR](http://multiplayar.me/) is a framework for building multi-client augmented reality experiences. This repository houses core infrastructure for a `multiplayar` deployment. This includes:
- Serves as a data transmission server
- Spins off image processing and transformation threads

Client code for iOS [can be found here](https://github.com/jacobkahn/multiplayar).

## Requirements

- [cmake](https://cmake.org/) - 2.8 or later
- [Boost](http://www.boost.org/) - 1.66.0 or later
- [OpenCV 3.4](https://opencv.org/opencv-3-4.html) - 3.4 or later
- [Crow](https://github.com/ipkn/crow) - included with the project in `include`

## Compiling and Running the Project

Build the project with `CMake` and `make`.

To the build project:
```
cd build
cmake ..
make
```

To run the project:
```
cd bin
./multiplayar-core
```