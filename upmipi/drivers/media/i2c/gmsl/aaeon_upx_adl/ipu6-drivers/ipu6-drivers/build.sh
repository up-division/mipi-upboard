#!/bin/bash

sudo dkms remove ipu6-drivers/0.0.0 --all
sudo dkms add .
sudo dkms build -m ipu6-drivers -v 0.0.0
sudo dkms autoinstall ipu6-drivers/0.0.0 --force