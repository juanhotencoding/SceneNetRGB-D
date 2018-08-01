# README #

### Related publications ###
Please cite this work if you make use of our system in any of your own endeavors:

* **[SceneNet RGB-D: Can 5M Synthetic Images Beat Generic ImageNet Pre-training on Indoor Segmentation?](http://www.imperial.ac.uk/media/imperial-college/research-centres-and-groups/dyson-robotics-lab/jmccormac_etal_iccv2017.pdf)**, *J. McCormac, A. Handa, S. Leutenegger, and A. J. Davison*, ICCV '17

Modifications by: Juan Vargas-Murillo
# 1. Overview #
The pipeline is divided into three components, each in their respective subfolder. 

* The scene\_generator randomly samples and positions objects, and runs a [physics simulation](https://projectchrono.org/) to produce a scene\_description.txt file. 
* The camera\_trajectory\_generator takes in this scene description and produces a camera trajectory using OpenGL z-buffer collision detection according to the two-body simulation, it then outputs a scene\_and\_trajectory\_description.txt.  
* Finally the renderer (a modified version of the [Opposite Renderer](https://github.com/apartridge/OppositeRenderer), using the [OptiX framework](https://developer.nvidia.com/optix)) takes in that text file and renders the trajectory, outputting rgb, depth, and instance mask.

# 2. How to setup #

For building each component see the README.md in the subdirectories. For running the software you will need
to download the following assets and extract them to some local path:

* ShapeNet Core dataset from [here](https://shapenet.org). (You will need to register)
* SceneNet layouts and textures from [here](https://github.com/jmccormac/SceneNetRGBD_Layouts.git)

E.g. '/path/to/your/ShapeNet/' should contain asset folders such as '02691156/40278c4bbf6c1f3d9642905e5096dbcb/models/model\_normalized.{obj,mtl}', and '/path/to/your/SceneNetRGBD_Layouts/' should contain the folders 'bathroom/bathroom1\_layout.obj' and 'texture\_library'

# 3. How to run it? #

The scenenet\_room\_generator.cpp allows you to select the train/val/test splits on line 53. It is set to train by default.
After building each of the components run:

```
cd scene_generator/build
./scenenet_room_generator /path/to/your/ShapeNet/ /path/to/your/SceneNetRGBD_Layouts/
cd ../../camera_trajectory_generator/build
./room_camera_intersection /path/to/your/ShapeNet/ /path/to/your/SceneNetRGBD_Layouts/ ../../scene_generator/build/scene_description.txt
cd ../../renderer/build
./scenenet_render /path/to/your/ShapeNet/ /path/to/your/SceneNetRGBD_Layouts/ ../../camera_trajectory_generator/build/scene_and_trajectory_description.txt
```

This should then produce framenum_{rgb.jpg,depth.png,instance.png} as well as a render_info.log file in the render build directory. If the rendering does not output anything it may say "Average intensity too extreme". This is because of the checks put in to ensure the images are not too bleached or dark.  Just keep rerunning the command and a new random seed will generate a different set of lights.

# 4. License #
SceneNet RGB-D is freely available for non-commercial use only.  Full terms and conditions which govern its use are detailed [here](http://www.imperial.ac.uk/dyson-robotics-lab/downloads/scenenet-rgbd/scenenet-rgbd-license/) and in the LICENSE.txt file.
