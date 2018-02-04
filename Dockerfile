FROM         ubuntu:14.04
CMD          bash

# Required system packages
RUN apt-get update && apt-get -y install \
    apt-utils \
    build-essential \
    curl \
    doxygen \
    git \
    tar \
    wget \
    xz-utils

# Install new CMake
RUN wget -q -O /tmp/cmake.tar.gz --no-check-certificate \
    https://cmake.org/files/v3.7/cmake-3.7.2-Linux-x86_64.tar.gz && \
    tar -xaf /tmp/cmake.tar.gz --strip-components=1 -C /usr/local && \
    rm /tmp/cmake.tar.gz

# Install new Clang
RUN apt update && apt install -y \
    xz-utils \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/* \
    && curl -SL http://releases.llvm.org/5.0.1/clang+llvm-5.0.1-x86_64-linux-gnu-ubuntu-16.04.tar.xz \
    | tar -xJC . && \
    mv clang+llvm-5.0.1-x86_64-linux-gnu-ubuntu-16.04 clang_5.0.1 && \
    echo 'export PATH=/clang_5.0.1/bin:$PATH' >> ~/.bashrc && \
    echo 'export LD_LIBRARY_PATH=/clang_5.0.1/lib:LD_LIBRARY_PATH' >> ~/.bashrc


# Install Boost
RUN sudo apt-get install libboost-all-dev

# Download and install OpenCV
RUN wget https://github.com/opencv/opencv/archive/3.4.0.zip && \
    unzip 3.4.0.zip && \
    cd opencv-3.4.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    sudo make -j4 install


