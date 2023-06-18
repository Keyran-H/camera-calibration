# Ceres & OpenCV bundle adjustment solution 
This project presents a Ceres based approach for camera calibration. TODO: Add a small blurb of how this repo is helpful and then show some evidence of my working code.
## Running the code

### Installing Dependencies 

Globablly installed [CERES](http://ceres-solver.org/installation.html#linux) and [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html) libraries are used in this project. Specify the PATH variable in CMakeLists.txt if a local version is desired to be used. The software versions used are:
- OS: Ubuntu 18.04
- CERES: 2.1.0
- OpenCV: 3.2.0 

### Building & Compiling

Generate the build files by running the top level CMakeList.txt within the build directory:
`cd build && cmake ..`

Compile the executable within the build directory: `make`

### Running

Execute the generated executable: `./nlo-camera-calibration`

The executable uses gflags which are documented in the source code. The defaults are set to use my images to demonstrate functionality. In summary, the parameters which can be changed are the initial estimates, calibration board parameters and reading/writing to disk options.
