#!/bin/bash

# Script that runs the commands needed to make the scene description files
# that will be used when generating the random camera trajectory

# Directories, Datasets, and File paths

# Location of the pipeline
SCENENET_DIR=

# Location of scene generator
SCENE_GENERATOR_DIR=

# location of ShapeNet Database
SHAPENET=

# Location of SceneNet RGB-D scene layouts
LAYOUTS=

# Number of scenes you want to generate
NUM_SCENES=100

for (( c=0; c<$NUM_SCENES; c++ ))
do
	echo "Creating Scene Number ${c}"

	# Start in ~/
	cd

	# Move to scene generator
	cd $SCENE_GENERATOR_DIR

	echo "Starting Scene Generator"
	
	# Create a file to store the output of terminal
	touch scene_output_${c}.txt

	# Run scenenet_room_generator until it returns a successful exit status
	until ./scenenet_room_generator $SHAPENET $LAYOUTS $c >> scene_output_${c}.txt
	do
		echo "ERROR...REGENERATING SCENE..."
	done

	echo "Done generating scene description file"

	# Make a directory to store scene info

	if [ ! -d "scenes" ]
	then
		mkdir scenes
		mkdir scenes/scene_${c}
		mv scene_description_${c}.txt scenes/scene_${c}/
		mv scene_output_${c}.txt scenes/scene_${c}/
	else
		mkdir scenes/scene_${c}
		mv scene_description_${c}.txt scenes/scene_${c}/
		mv scene_output_${c}.txt scenes/scene_${c}/
	fi

done

exit
