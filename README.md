3D Scanner with OpenNI and Point Cloud Library
========================

Basic 3D Scanner is a simple, cross-platform 3D scanning solution using OpenNI compatible devices Kinect for XBOX, Asus XMotion and Prime Sense Carmine. Basic 3D Scanner is developed as a term project for CSE395 lecture in Computer Engineering of Gebze Institute of Technology and should not be considered as a final 3D Scanner solution.

For more information checkout the project's website: http://furkantektas.github.io/basic-3D-scanner/

## Requirements ##
* PCL 1.6+
* Qt 4.8.x
* QtVTK
* Boost 
* Eigen

## How to compile ##
Create a folder 'build'.
```
mkdir build
```

Navigate to build folder and create a makefile using cmake.
```
cd build
cmake ..
```

Compile using make
```
make
```

## How to use ##
After a successful compiling process, you can run ScannerProje executable as below.

```
./ScannerProject 
```