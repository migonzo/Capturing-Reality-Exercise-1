# Capturing Reality Framework
This is the basic framework for all Capturing Reality exercises. Please implement all your stuff in the `exercise` folder. The image class [is documented here](./doc/image.md).

# How to build (on Linux)
```
mkdir build
cmake ..
make -j4
```

```
./exercise/ex ~/Pictures/SomeImageFile.jpeg
```

Please take care to install the following dependencies:

* libpng
* jpeglib
* OpenEXR
* Eigen
* Ceres (for the data fitting exercise)

...using the following commands:

* Debian and Ubuntu: `sudo apt-get install libpng-dev libjpeg-dev libopenexr-dev libeigen3-dev ceres-solver`
* Arch Linux: `sudo pacman -S libpng libjpeg openexr eigen3`, `ceres-solver` from AUR

# Tow to build (on Windows)
* Download the `3rdparty` archive and extract it to the repository root
* Run the CMake GUI
* Set the source directory to the repository root
* Set the build folder to [repo root]/build
* Run `Configure` and `Generate`
* Click `Open Project` or open `CRFramework.sln`
* Build the project using Visual Studio