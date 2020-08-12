# Monocular Camera Calibration

Set the path of checkerboard images in src/Camera_Calibration.cpp file.

Set the checkerboard parameters in  src/Camera_Calibration.cpp file (width, height and size (in metres))

Set calibration flags

Run Calibration.

# Build 

git clone https://github.com/deepak-1530/Photogrammetry

cd Photogrammetry

mkdir build

cd build

cmake ..

make -j$(nproc)
