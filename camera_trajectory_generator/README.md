# README #

This is the physics simulation code used to generate random scene description textfiles used in the paper:

* **[SceneNet RGB-D: Can 5M Synthetic Images Beat Generic ImageNet Pre-training on Indoor Segmentation?](http://www.imperial.ac.uk/media/imperial-college/research-centres-and-groups/dyson-robotics-lab/jmccormac_etal_iccv2017.pdf)**, *John McCormac, Ankur Handa, Stefan Leutenegger, Andrew J Davison*, ICCV '17

# 1. How to build it? #

The following instructions have been tested on Arch linux, with gcc 7.2.0, CUDA 8.0.  For the CVD dependency a compiler that supports C++14 is required.

Dependencies are:

* CMake
* OpenGL
* GLUT
* [CUDA >= 8.0](https://developer.nvidia.com/cuda-downloads)
* [CVD](https://github.com/edrosten/libcvd) 
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
* [ASSIMP](http://assimp.sourceforge.net/)

NOTE: For CVD make sure to compile with at least some of the optional dependencies such as libjpeg and libpng - if you do not have these you may experience errors when rendering such as:

```
terminate called after throwing an instance of 'CVD::Exceptions::Image_IO::UnsupportedImageSubType'
Aborted (core dumped)
```

Then build normally. I.e.:

```
mkdir build
cd build
cmake ..
make
```
