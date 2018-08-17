# Running on BRUTE
There are bash scripts to automate the process of generating scene generation, camera trajectory generation and rendering. They are located in ```/home/jvm/Documents/scenenet``` and are the following scripts:
- ```generate_scenes.sh```
    - takes a command line argument for the amount of scenes to create
- ```generate_trajectories.sh```
- ```render_scenes.sh```
- ```run_pipeline.sh```
    - takes a command line argument fo the amount of scenes to create

Run the ```run_pipeline.sh``` script as follows
```
/home/jvm/Documents/scenenet/run_pipeline.sh N (# number of scenes)
```

This will go about making the scene, trajectory, rendering the images and making the json file, instance.json.