cmake_minimum_required(VERSION 2.8)
project(multiplayar-core)
MESSAGE("Initializing install...")

# Set C++ version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Compilation options
add_compile_options(-Wall -g)

# Add base executable
include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(multiplayar-core ${PROJECT_SOURCE_DIR}/src/main.cpp)

# Find Boost
MESSAGE("Locating dependency: Boost")
find_package(Boost COMPONENTS system filesystem REQUIRED)
if(Boost_FOUND)
    MESSAGE("Found dependency: Boost")
    include_directories(${Boost_INCLUDE_DIRS}) 
endif()


# Find OpenCV
MESSAGE("Locating dependency: OpenCV")
FIND_PACKAGE(OpenCV REQUIRED)
if (OpenCV_FOUND)
    MESSAGE("Found dependency: OpenCV")
    include_directories(${OpenCV_INCLUDE_DIRS}) 
endif()

# Include directory
include_directories(${PROJECT_SOURCE_DIR}/include)

########## Core Assets ##########
# Server
add_library(
    server
    src/server/Server.cpp
)

add_library(
    environment
    src/environment/Client.cpp
    src/environment/Entity.cpp
    src/environment/Environment.cpp
    src/environment/Object.cpp
)

add_library(
    cv
    src/cv/SIFT.cpp
    src/cv/SIFTWriter.cpp
)

# Output
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

# Includes from project
include_directories("${PROJECT_SOURCE_DIR}")

# Link dependencie with main executable
target_link_libraries(
    multiplayar-core
    server
    environment
    cv
    -lpthread
    -lm
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)