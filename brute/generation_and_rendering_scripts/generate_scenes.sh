#!/bin/bash

# Script that runs the commands needed to make a scene description file
# that will be used when making the camera trajectory file

# Directories, Datasets, and File Paths
SCENENET_DIR=Documents/scenenet/src
SCENE_GENERATOR_DIR=$SCENENET_DIR/scene_generator/build
SHAPENET=../../../ShapeNetCore.v2/
SCENE_LAYOUTS=../../../SceneNetRGBD_Layouts/
SCENE_DATASET=../../../scene_dataset
OLD_SCENES=`find ../scenes/* -maxdepth 0 -type d | wc -l`
NEW_SCENES=$1

if [ "$NEW_SCENES" != "" ]; then
        echo "Creating ${NEW_SCENES} New Scenes"
else
        echo "Please provide the amount of new scenes you wish to create" 
        read NEW_SCENES
        echo "Creating ${NEW_SCENES} New Scenes"
fi
# if we don't have a file, start at zero
if [ ! -f "scenes.dat" ] ; then
  value=${OLD_SCENES}

# otherwise read the value from the file
else
  value=`cat scenes.dat`
fi

# increment the value
value=`expr ${value} + ${NEW_SCENES}`

# show it to the user
# echo "value: ${value}"
echo "For a total of ${value} scenes"

# and save it for next time
echo "${value}" > scenes.dat


for (( c=0; c<${value}; c++))

do
	# Start in ~/
	cd 

	# First Component, Generate Scene
	cd $SCENE_GENERATOR_DIR

	if [ -d "../../../../scenes/scene_${c}" ]
	then 
		echo "Scene ${c} Already Exists"
		continue
	else
		echo "Creating Scene Number $c"
		echo "Starting Scene Generator"
		touch scene_output_${c}.txt
		until ./scenenet_room_generator $SHAPENET $SCENE_LAYOUTS $c > scene_output_${c}.txt
	
		do
			echo "ERROR...REGENERATING SCENE..."
		done

		echo "Done generating scene description file"


		if [ ! -d "../../../../scenes" ]
	
		then
			mkdir ../../../../scenes
			mkdir scenes/scene_${c}
			mv scene_description_${c}.txt ../../../../scenes/scene_${c}/
			mv scene_output_${c}.txt ../../../../scenes/scene_${c}/

		else
			mkdir ../../../../scenes/scene_${c}
			mv scene_description_${c}.txt ../../../../scenes/scene_${c}/
			mv scene_output_${c}.txt ../../../../scene_descriptions/scene_${c}/
		fi

		echo "Moved file to scene descriptions folder"
	fi

done

exit
