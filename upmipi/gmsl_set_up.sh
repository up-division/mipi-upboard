sudo modprobe max96724_driver
sudo i2cset -f -y 16 0x27 0x00 0x00 0x42 i
sudo modprobe max96717_driver
echo "max96717 0x21" | sudo tee /sys/bus/i2c/devices/i2c-1/new_device