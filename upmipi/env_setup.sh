#!/bin/bash
RED='\033[0;31m'
NC='\033[0m' # No Color/Reset

## kernel limit 6.14+
is_ver_over(){
    return $(uname -r | awk -F '.' '{
        if ($1 < 6) { print 1; }
        else if ($1 == 6) {
            if ($2 < 14) { print 1; }
            else { print 0; }
        }
        else { print 0; }
    }')
}

## check kernel version
if is_ver_over
then
    echo $(uname -r)
else
    echo "Kernel version is less than 6.14"
    exit
fi

## build & install kernel modules
sudo apt install build-essential
make
if [ $? -ne 0 ]
then
    make clean
    echo -e "\n\n${RED} build error occurred! fix it and try again !"
    exit
fi
sudo make modules_install
make clean
cd ..

## Debian requirement
sudo apt install \
libgstreamer-plugins-base1.0-dev \
libgstreamer-plugins-base1.0-0 \
gstreamer1.0-plugins-good \
libgstreamer-plugins-good1.0-dev \
gstreamer1.0-plugins-bad-apps \
gstreamer1.0-plugins-bad \
libgstreamer-plugins-bad1.0-0 \
gstreamer1.0-plugins-ugly \
libexpat1-dev libdrm-dev v4l-utils yavta python-is-python3 python3-venv

sudo usermod -aG video $USER
sudo usermod -aG render $USER
sudo depmod -a

echo -e "${RED}Please reboot your system.......${NC}" 



