#!/bin/bash
source /etc/os-release || echo 'Warning: /etc/os-release was not found'

if [[ " $ID_LIKE " == *' archlinux '* ]]; then
  sudo pacman -S git libuv openssl gcc cmake make
else
  if [[ ! " $ID_LIKE " == *' debian '* ]]; then
    echo 'Warning: unidentified Linux distribution, assuming Debian-like'
  fi

  sudo apt-get update
  sudo apt-get -y install git 
  sudo apt-get -y install libuv1-dev 
  sudo apt-get -y install libssl-dev 
  sudo apt-get -y install gcc 
  sudo apt-get -y install g++ 
  sudo apt-get -y install cmake 
  sudo apt-get -y install make
  sudo apt-get -y install openssl
  sudo apt-get -y install zlib1g-dev
fi

git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
