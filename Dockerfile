FROM         ubuntu:trusty
CMD          bash

# Required system packages
RUN apt-get update && apt-get -y install \
    apt-utils \
    build-essential \
    curl \
    doxygen \
    git \
    tar \
    wget

# Core dependencies
RUN apt-get install cmake \
    clang \
    libboost-all-dev

WORKDIR /
# Download and install OpenCV
RUN wget https://github.com/opencv/opencv/archive/3.4.0.zip -O opencv.zip && \
    unzip opencv.zip && \
    wget https://github.com/opencv/opencv_contrib/archive/3.4.0.zip -O opencv_contrib.zip && \
    unzip opencv_contrib.zip && \
    mkdir opencv-3.4.0/build && \
    cd opencv-3.4.0/build && \
    cmake -DOPENCV_EXTRA_MODULES_PATH=/opencv_contrib-3.4.0/modules/ \
    -DBUILD_opencv_python3=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_opencv_java=OFF .. && \
    sudo make -j4 install && \
    rm /opencv.zip && \
	rm /opencv_contrib.zip && \
	rm -r /opencv-3.4.0 && \
	rm -r /opencv_contrib-3.4.0
