#!/bin/bash

# Script that runs the commands needed to make a scene and trajectory description file
# that will be used when rendering the scene

# Directories, Datasets, and File Paths
SCENENET_DIR=Documents/scenenet/src
SCENE_GENERATOR_DIR=$SCENENET_DIR/scene_generator/build
SHAPENET=../../../ShapeNetCore.v2/
SCENE_LAYOUTS=../../../SceneNetRGBD_Layouts/
SCENE_DATASET=../../../scene_dataset
SCENE_DESCRIPTIONS=../../scene_generator/build/scene_descriptions/
CAMERA_TRAJECTORY_GENERATOR_DIR=$SCENENET_DIR/camera_trajectory_generator/build
NUM_TRAJS=1

for (( c=0; c<$NUM_TRAJS; c++))
do
    cd 

    cd $CAMERA_TRAJECTORY_GENERATOR_DIR

    for f in $(find $SCENE_DESCRIPTIONS -name 'scene_description_*.txt') 
    
        do
        

        scene_id=$(echo $f | cut -d '_' -f 6 | sed -e s/[^0-9]//g)
        
        echo "Scene Description File Path: $f" "Scene ID:$scene_id" "Trajectory ID: $c"
        if [ -f "$SCENE_DESCRIPTIONS/scene_${scene_id}/scene_and_trajectory_description_${scene_id}_${c}.txt" ]
        then
            echo "Trajectory Already Exists For Scene ${scene_id} Moving To Next Scene"
            continue
        else
            echo "Generating Camera Trajectory"     
            echo "Creating scene_and_trajectory file"
            # touch scene_and_trajectory_${scene_id}_${c}.txt
        
            # touch traj_output_scene${scene_id}_${c}.txt 
        
            until ./room_camera_intersection $SHAPENET $SCENE_LAYOUTS ${f} ${scene_id} ${c}  # > traj_output_scene${scene_id}_${c}.txt
            do
                echo "ERROR...REGENERATING TRAJECTORY...."  
            done

            mv scene_and_trajectory_description_${scene_id}_${c}.txt $SCENE_DESCRIPTIONS/scene_${scene_id}/
        fi 
    done


done 

echo "DONE"

exit
