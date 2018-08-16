# Building from source
If you would like to build the SceneNet RGB-D dataset generation pipeline from scratch on your workstation, look [here](https://github.com/juanhotencoding/SceneNetRGB-D/tree/master/src) for more information. 
# Running on BRUTE (recommended)
If you would like to run on BRUTE, ssh in and run the generation and rendering scripts to the amount of scenes you wish to create. Then do post processing on the images to obtain filtered images, bounding box, pixel area, rle mask, and add to JSON.
# Running on HPC Summit
If you would like to use the Research Computing resources offered at CU-Boulder, the scripts needed to run the data generation pipeline are found here, you can submit a job with these scripts as the content. There is a containerized version of the pipeline on the computing cluster.