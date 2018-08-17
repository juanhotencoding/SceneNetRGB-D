#!/bin/bash

# Script that runs the commands needed to render scenes
# and make the instance, depth, and photo images.

# Directories, Datasets, and File Paths
SCENENET_DIR=Documents/scenenet/src
SCENE_GENERATOR_DIR=$SCENENET_DIR/scene_generator/build
CAMERA_TRAJECTORY_GENERATOR_DIR=$SCENENET_DIR/camera_trajectory_generator/build
RENDERER_DIR=$SCENENET_DIR/renderer/build
SHAPENET=../../../ShapeNetCore.v2/
SCENE_LAYOUTS=../../../SceneNetRGBD_Layouts/
SCENE_FILE=../../scene_generator/build/scene_description.txt
SCENE_AND_TRAJECTORY_FILE=../../camera_trajectory_generator/build/scene_and_trajectory_description.txt
SCENE_DATASET=../../../scene_dataset
SCENE_DESCRIPTIONS=../../scene_generator/build/scene_descriptions/

cd 

cd $RENDERER_DIR

for f in $(find $SCENE_DESCRIPTIONS -name 'scene_and_trajectory_description*.txt')
do 
    echo "Looping through Scene and Trajectory description files"
    
    scene_id=$(echo $f | cut -d '_' -f 8)
    traj_id=$(echo $f | cut -d '_' -f 9 | sed -e s/[^0-9]//g)
    
    if [ -f "$SCENE_DESCRIPTIONS/scene_${scene_id}/render_info_${scene_id}_${traj_id}.log" ]
    then
        echo "Already Rendered Scene ${scene_id}"
        echo "Rendering Next Scene"
        continue
    else
        echo "Rendering Scene ID:${scene_id} Trajectory ID:${traj_id}"
        
        # cat $SCENE_DESCRIPTIONS/scene_${scene_id}/scene_and_trajectory_description_${scene_id}_${traj_id}.txt
        # echo $f ${scene_id} ${traj_id}

        mkdir $SCENE_DESCRIPTIONS/scene_${scene_id}
        mkdir $SCENE_DESCRIPTIONS/scene_${scene_id}/photo
        # mkdir $SCENE_DESCRIPTIONS/scene_${scene_id}/depth
        mkdir $SCENE_DESCRIPTIONS/scene_${scene_id}/instance

        echo "Finished making directories to hold images"

        until ./scenenet_render $SHAPENET $SCENE_LAYOUTS ${f}
        do
            echo "ERROR...RERENDERING SCENE"
        done 

        # We also want to make a render_info_log_${scene_id}_${traj_id}
        cp render_info.log render_info_${scene_id}_${traj_id}.log
        mv render_info_${scene_id}_${traj_id}.log $SCENE_DESCRIPTIONS/scene_${scene_id}

        mv *jpg $SCENE_DESCRIPTIONS/scene_${scene_id}/photo
        # mv *depth.png $SCENE_DESCRIPTIONS/scene_${scene_id}/depth
        mv *instance.png $SCENE_DESCRIPTIONS/scene_${scene_id}/instance
    fi
    
done 

exit 