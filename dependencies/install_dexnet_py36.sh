#!/bin/bash

# read cmd line inputs
VERSION=$1 # cpu or gpu

# set cpu/gpu conditional libraries
case "${VERSION}"
in
cpu)
	sudo python3.6 -m pip install tensorflow==1.15.0
	;;
gpu)
  sudo python3.6 -m pip install tensorflow-gpu==1.13.1
	;;
*)
	echo "Usage: $0 {cpu|gpu}"
	exit 1
esac

# install apt deps
#sudo apt install cmake libvtk6-dev python3-vtk6 python3-sip python3-qt4 libosmesa6-dev meshlab libhdf5-dev
sudo apt install cmake libvtk6-dev python3-vtk7 python3-sip  libosmesa6-dev meshlab libhdf5-dev python3.6-tk -y

# install pip deps
sudo python3.6 -m pip install --force -r dexnet_requirements.txt
sudo python3.6 -m pip install --force kiwisolver
python3.6 -m pip install --user --force psutil

# install deps from source
mkdir dexnet_deps
cd dexnet_deps

# install autolab modules
git clone https://github.com/BerkeleyAutomation/autolab_core.git
git clone https://github.com/BerkeleyAutomation/perception.git
git clone https://github.com/BerkeleyAutomation/gqcnn.git
git clone https://github.com/BerkeleyAutomation/visualization.git

cd autolab_core
git checkout d4d288c
sudo python3.6 -m pip uninstall autolab_core
sudo python3.6 -m pip install --force -e .
cd ..

cd perception
git checkout e1c936f
sudo python3.6 -m pip uninstall perception
sudo python3.6 -m pip install --force -e .
cd ..

cd gqcnn
git checkout a5f9353
sudo python3.6 -m pip uninstall gqcnn
sudo python3.6 -m pip install --force -e .
cd ..

cd visualization
git checkout 0648818
sudo python3.6 -m pip uninstall visualization
sudo python3.6 -m pip install --force -e .
cd ..
