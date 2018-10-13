# MAAV Software

---

## Table Of Contents

---

## GNC

Guidance, navigation, and control.

### Setup and Running

`./scripts/setup.sh`

Set the MAAV_SOFTWARE_DIR environment variable on your machine to the path to the maav software
folder on your machine. Add export MAAV_SOFTWARE_DIR='path' to your shell config.

### Compile the code:

`mkdir build`

`cd build`

`cmake ..`

`make -j`

### Download the TUMs dataset (GNC):

https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz

extract into `datasets/`

### Run the example (GNC):

`./bin/tums Vocabulary/ORBvoc.txt tests/tums/TUM1.yaml datasets/rgbd_dataset_freiburg1_xyz tests/tums/associations/fr1_xyz.txt`

---

## Perception
