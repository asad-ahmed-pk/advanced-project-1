# Advanced Project I
## Drone Based Vegetation Monitoring and Analysis

This is the accompanying developed program for the Advanced Project I module submission at Jacobs University Bremen. Part of the Data Engeering MSc Programme.

## Getting Started

Clone this repo using git and build using CMake. Please refer to the prerequisites below for the required dependancies.

### Prerequisites

This project is a C++ 11 project using the CMake build system. The following dependancies are required:

1. Point Cloud Library (PCL)
2. PDAL
3. Eigen
4. FLANN

### Installing

After installing the dependancies.

```
mkdir build
cd build
cmake ..
make
```

Once compiled, run:

```
./PointCloudTreeClassifier [input point cloud - LAS|PCD|PLY file path] (optional - number of threads to use for reading LAS file concurrently)
```

