3D Scanner with Kinect for XBOX
========================

This project uses Kinect for XBOX and a basic stepper motor for scanning objects 
as 3D meshes and clouds.

For each iteration, clouds are saved as icp-#.pcd, final cloud is saved as 
FinalCloud.pcd, final mesh is saved as FinalMesh.ply.

Object will be scanned should be in the clipping box and the red line in the
clipping box should be cross in the center of the object.

## Requirements ##
* PCL 1.6+
* Qt 4.8.x
* QtVTK
* Boost 
* Eigen

## How to compile ##
Create a folder named as build.
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
./ScannerProje 
```