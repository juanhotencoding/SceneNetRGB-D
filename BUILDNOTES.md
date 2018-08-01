## Overall Tips

If you are building on an Ubuntu system 16> it is best to compile using gcc-6 you can install a different compiler and switch between versions by doing the following

```
sudo apt-get install gcc-6 g++-6


# Set gcc and g++ to compile using the same version
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 60 --slave /usr/bin/g++ g++ /usr/bin/g++-6

# Allows you to choose which gcc compiler version you want to use
sudo update-alternatives --config gcc
```
