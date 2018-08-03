#!/bin/bash

# Script that runs the commands needed to make the 
# scene and trajectory description file that will
# be used in the rendering step

# Directories, Datasets, and File paths
SCENENET_DIR=
SHAPENET=
LAYOUTS=
SCENES=
CAMERA_TRAJECTORY_GENERATOR_DIR=
NUM_TRAJS=1

for (( c=0; c<NUM_TRAJS; c++))
do 
    cd 
    cd $CAMERA_TRAJECTORY_GENERATOR_DIR

    for f in $(find $SCENES -name 'scene_description_*.txt')
    do 
        scene_id=$(echo $f | cut -d '_' -f 6 | sed -e s/[^0-9]//g)

        echo "Scene Description File Path: $f" "Scene ID: $scene_id" "Trajectory ID: $c"
        if [ -f "$SCENES/scene_${scene_id}/scene_and_trajectory_description_${scene_id}_${c}.txt" ]
        then
            echo "Trajectory Already Exists For Scene ${scene_id}"
            echo "Moving On To Next Scene"
            continue
        else
            echo "Generating Camera Trajectory"
            echo "Creating scene_and_trajectory_description file"

            until ./room_camera_intersection $SHAPENET $LAYOUTS ${f} ${scene_id} ${c}
            do
                echo "ERROR...REGENERATING TRAJECTORY..."
            done
            mv scene_and_trajectory_description_${scene_id}_${c}.txt $SCENES/scene_${scene_id}/
        fi    

    done
done