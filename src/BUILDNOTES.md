## Overall Tips

If you are building on an Ubuntu system 16> it is best to compile using gcc-6 you can install a different compiler and switch between versions by doing the following

```
sudo apt-get install gcc-6 g++-6


# Set gcc and g++ to the same version
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6

# Allows you to choose which gcc compiler version you want to use
sudo update-alternatives --config gcc
```


When building the dependencies that each component needs, use the repositories found [here](https://github.com/ankurhanda?tab=repositories), namely, these are the Pangolin, CVD, and TooN.

It is also a good idea to install the dependencies that these repositories rely on as well, the required dependencies can be found in **requirements.txt** and can be run as followed

```
cat requirements.txt | xargs sudo apt-get -y install
```

### Building the Scene Generator
The scene generator makes use of the open source physics engine, Project Chrono, there is a google groups forum post which shows you how to build Project Chrono [here](https://groups.google.com/forum/#!topic/projectchrono/mcF5h35ILWI).

