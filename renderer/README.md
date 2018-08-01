# README #

This is the rendering code used to generate random scene description textfiles used in the paper:

* **[SceneNet RGB-D: Can 5M Synthetic Images Beat Generic ImageNet Pre-training on Indoor Segmentation?](http://www.imperial.ac.uk/media/imperial-college/research-centres-and-groups/dyson-robotics-lab/jmccormac_etal_iccv2017.pdf)**, *John McCormac, Ankur Handa, Stefan Leutenegger, Andrew J Davison*, ICCV '17

It is a modified version of the Opposite Renderer available [here](https://github.com/apartridge/OppositeRenderer).  A GPU with 8GB of memory was used for rendering.

# 1. How to build it? #

The following instructions have been tested on Arch linux, with gcc 7.2.0, CUDA 8.0.  For the CVD dependency a compiler that supports C++14 is required. Additionally for the cuda code the host compiler has been set to gcc-5 as gcc-7 is not supported.  It is also possible to use gcc-6 in which case the ```SET(CUDA_HOST_COMPILER gcc-5)``` line in ```CMakeLists.txt``` will need to be updated to the installed version.

Dependencies are:

* CMake
* OpenGL
* GLUT
* [CUDA >= 8.0](https://developer.nvidia.com/cuda-downloads)
* [CVD](https://github.com/edrosten/libcvd) (make sure to compile with at least some of the optional dependencies such as libjpeg and libpng.)
* [Pangolin](https://github.com/stevenlovegrove/Pangolin)
* [ASSIMP](http://assimp.sourceforge.net/)
* Eigen
* [Optix](https://developer.nvidia.com/optix)

Download and install the Optix framework (NVIDIA-OptiX-SDK-4.1.1) from the Nvidia website (you probably have to register with them) - point towards the folder it is installed in CMakeModules/FindOptiX.cmake.

```
set(OptiX_INSTALL_DIR "/your/folder/containing/Optix" CACHE PATH "Path to OptiX installed location.")
```

by default it assumes the Optix folder (i.e. containing lib64 and include directories), is in the main renderer folder (alongside the src directory)



## Make a build directory and build the project

Make and build the project with the normal cmake build pattern. I.e.

```
mkdir build
cd build
cmake ..
make
```

## Common issues

* If you receive errors such as 'The supplied PTX requires at least one device with SM 61 or greater', you need to edit the version in the CMakeLists.txt line: 18 to suit your GPU. Nvidia has a list [here](https://developer.nvidia.com/cuda-gpus). If you click on 'CUDA-Enabled GeForce Products' and find your GPU and its 'Compute Capability' e.g. For a GTX 770 it has Compute Capability 3.0 for which you would use 30 e.g. ```SET(CUDA_NVCC_FLAGS   "-arch=compute_30 -code=sm_30"  "--use_fast_math" "-O3" "-lineinfo" "-Xcompiler")```
* If you receive errors such as 'CVD::Exceptions::Image_IO::UnsupportedImageSubType' this is often as a result of the texture interlacing of pngs in ShapeNets.  You can convert the textures in the whole directory using ImageMagick and the following command: ```find ./ -type f -name "*.png" -exec convert -interlace none -define png:color-type=6 {} {} \;```
* If after converting the textures you still receive errors such as ```terminate called after throwing an instance of 'CVD::Exceptions::Image_IO::UnsupportedImageSubType``` ensure you have libjpeg and libpng installed as dependencies before compiling CVD.

## Other things


You can set the image resolution in the same scenenet\_render.cpp by
editing the WIDTH and HEIGHT variables

```
unsigned int BaseScene::WIDTH  = 320u;
unsigned int BaseScene::HEIGHT = 240u;
```

You can also change the quality of the rendering with the following variables
(higher number means higher quality)

```
m_renderer->setNumIterations(4);
m_renderer->setNumPhotonMaps(5);
```

Finally you can also edit the ppmRadius for the photon mapping to reduce the
circular artifacts (at the cost of slower rendering) in ```src/Renderer/OptixRenderer.cpp line 372```

```
  const float initppmRadius = 0.05; //0.01 is the original scenenet parameter
```
