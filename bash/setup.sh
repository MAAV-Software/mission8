git remote add upstream git@gitlab.eecs.umich.edu:maav/gnc.git

sudo apt install clang-format -y
sudo apt install yaml-cpp -y
sudo apt install libeigen3-dev -y
sudo apt install libopencv-dev -y
sudo apt install curl -y

git submodule update --init

cd /tmp
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build 
cd build
cmake ..
make -j
sudo make install

echo('Romario please do bash')
# TODO:
# curl https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
# unzip this into gnc/datasets

#TODO:
# Remove Vocabulary/ORBvoc.txt
# Unzip ORBvoc.txt.tar.gz