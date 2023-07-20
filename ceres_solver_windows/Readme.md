 - In codeBlocks: Project > Build options > Search directories -> 
    - D:\MY_DRIVE_N\Masters_thesis\ceres\eigen-3.4.0\bin
    - D:\MY_DRIVE_N\Masters_thesis\ceres\gflags\bin
    - D:\MY_DRIVE_N\Masters_thesis\ceres\glog
    - D:\MY_DRIVE_N\Masters_thesis\ceres\ceres-solver\ceres-bin
    - D:\MY_DRIVE_N\Masters_thesis\ceres\ceres-solver\ceres-bin\include
    - D:\MY_DRIVE_N\Masters_thesis\ceres\eigen-3.4.0\Eigen\src
    - D:\MY_DRIVE_N\Masters_thesis\ceres\eigen-3.4.0
    - D:\MY_DRIVE_N\Masters_thesis\ceres\glog\bin
    - D:\MY_DRIVE_N\Masters_thesis\ceres\glog\src
    - D:\MY_DRIVE_N\Masters_thesis\ceres\gflags\bin\include
    - D:\MY_DRIVE_N\Masters_thesis\ceres\ceres-solver\ceres-bin\examples\slam

## Steps
- Follow (http://ceres-solver.org/installation.html#windows)
- Requirement :
    - C++ > 17
    - MinGW
    - cmake
    - eigen-3.4
    - glog
    - gflags
- Download and install(cmake) all requirements
- Set cmake enviornments:
    - Eigen3_DIR -> D:\MY_DRIVE_N\Masters_thesis\ceres\eigen-3.4.0\bin
    - gflags_DIR -> D:\MY_DRIVE_N\Masters_thesis\ceres\gflags\bin
    - glog_DIR -> D:\MY_DRIVE_N\Masters_thesis\ceres\glog\bin


## Issues
- Build successful with cmake
- Cannot build in codeBlocks
    ```
    ||error: can't write 111 bytes to section .text of obj\Debug\examples\more_garbow_hillstrom.o: 'File too big'|
    ||error: can't close obj\Debug\examples\more_garbow_hillstrom.o: File too big|
    ```
