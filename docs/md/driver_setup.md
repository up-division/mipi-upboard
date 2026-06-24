# <b> GMSL Driver Setup Guide </b>

## <b> Ubuntu 24.04 LTS Environment </b>

This guide is intended for systems running:

* **Operating System:** Ubuntu 24.04 LTS
* **Linux Kernel:** 6.17
* **Target Platform:** UP Board
* **Supported GMSL Hardware:**

  * MAX96724 / MAX96724F Deserializer
  * MAX96717F Serializer
  * SG3S-ISX031C-GMSL2F Camera Module

Verify the operating system and kernel version before continuing:

```bash
cat /etc/os-release
uname -r
```

Example output:

```text
Ubuntu 24.04 LTS
6.17.x
```

> **Important — Lock the kernel version:**
>
> The IPU, MIPI, and GMSL drivers are built for a specific Linux kernel version. After confirming that the camera works correctly, lock the kernel packages to prevent automatic kernel updates.
>
> If the expected GMSL or MIPI camera messages suddenly disappear from `dmesg`, check `uname -r` first. The system may have automatically updated and booted into a kernel for which the camera drivers have not been built.

---

## <b> Development Kit Installation </b>

Update the package database and install the required development tools, I²C utilities, GPIO utilities, V4L2 tools, GStreamer packages, kernel headers, and DKMS:

```bash
sudo apt update
sudo apt -y upgrade

sudo apt install -y \
  build-essential \
  git \
  pkg-config \
  python3 \
  python3-pip \
  python3-venv \
  i2c-tools \
  gpiod \
  v4l-utils \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  libdrm-dev \
  linux-headers-$(uname -r) \
  dkms \
  yavta
```

If the kernel is upgraded during this process, reboot the system before building the drivers:

```bash
sudo reboot
```

After rebooting, verify that the kernel headers match the running kernel:

```bash
uname -r
ls -ld /lib/modules/$(uname -r)/build
```

The `/lib/modules/<kernel-version>/build` path should exist and point to the installed kernel headers.

### <b> Verify Installed Utilities </b>

```bash
i2cdetect -V
gpiodetect --version
v4l2-ctl --version
gst-launch-1.0 --version
dkms --version
```

List the available I²C buses, GPIO controllers, and video devices:

```bash
i2cdetect -l
gpiodetect
v4l2-ctl --list-devices
```

---

## <b> BIOS Configuration </b>

The BIOS camera configuration must match the connected camera module and the physical CSI port.

Refer to the corresponding configuration guide:

* [AR0234 Camera Configuration](https://github.com/up-division/mipi-upboard/wiki/Ar0234-Settings)
* [OV5647 — Raspberry Pi Camera Module 1](https://github.com/up-division/mipi-upboard/wiki/OV5647_Settings)
* [IMX219 — Raspberry Pi Camera Module 2](https://github.com/up-division/mipi-upboard/wiki/Imx219_Settings)
* [IMX477 — Raspberry Pi High Quality Camera](https://github.com/up-division/mipi-upboard/wiki/Imx477-Settings)
* [IMX708 — Raspberry Pi Camera Module 3](https://github.com/up-division/mipi-upboard/wiki/Imx708_Settings)
* [GMSL — MAX96724 + MAX96717F + ISX031](https://github.com/up-division/mipi-upboard/wiki/GMSL%28MAX96724f-MAX96717f-ISX031%29_Settings)

> **Important:**
> Incorrect BIOS CSI or I²C settings may prevent the camera, serializer, or deserializer from being detected by Linux.

After changing the BIOS configuration, save the settings and reboot the system.

---

## <b> Build and Install the IPU, MIPI, and GMSL Drivers </b>

Clone the UP Board MIPI driver repository:

```bash
git clone https://github.com/up-division/mipi-upboard.git
```

Enter the driver directory:

```bash
cd mipi-upboard/upmipi
```

Make the environment setup script executable:

```bash
chmod +x env_setup.sh
```

Run the setup script with administrator privileges:

```bash
sudo ./env_setup.sh
```

Reboot the system after the installation is completed:

```bash
sudo reboot
```

Driver modules (`*.ko`) are typically installed under `/lib/modules/$(uname -r)/updates/` or one of its subdirectories.

### <b> Verify the Driver Installation </b>

After rebooting, check whether the GMSL-related kernel modules are loaded:

```bash
lsmod | grep -E "max96724|max_gmsl"
```

Check the kernel log for IPU, MIPI, GMSL, serializer, deserializer, and camera messages:

```bash
sudo dmesg | grep -Ei "ipu|mipi|gmsl|max96724|max96717|isx031"
```

Check whether video devices have been created:

```bash
v4l2-ctl --list-devices
ls -l /dev/video*
```

Check whether media-controller devices are available:

```bash
ls -l /dev/media*
```

Display the media-controller topology:

```bash
sudo media-ctl -p
```

---

## <b> Platform Port Mapping </b>

The CSI ports and I²C buses used for camera connections depend on the UP Board platform.

| Platform    | CSI Port | I²C Bus |
| ----------- | -------: | ------: |
| Alder Lake  |    1 / 2 |   1 / 5 |
| Twin Lake   |    1 / 2 |   1 / 5 |
| Meteor Lake |    0 / 4 |   0 / 5 |
| Arrow Lake  |    0 / 4 |   0 / 5 |

The actual port and bus depend on the physical connector, BIOS configuration, and ACPI camera node.



---

## <b> Default I²C Device Addresses </b>

The following table lists the commonly used Linux 7-bit I²C addresses.

| Device               | Device Type  | Default 7-bit I²C Address |
| -------------------- | ------------ | ------------------------: |
| IMX708               | Image Sensor |                    `0x1A` |
| MAX96724 / MAX96724F | Deserializer |                    `0x27` |
| MAX96717F            | Serializer   |                    `0x40` |
| ISX031               | Image Sensor |                    `0x1A` |

### <b> GMSL Device Terminology </b>

| Abbreviation  | Meaning                      | Device               |
| ------------- | ---------------------------- | -------------------- |
| DES           | Deserializer                 | MAX96724 / MAX96724F |
| SER           | Serializer                   | MAX96717F            |
| CAM           | Camera / Image Sensor        | ISX031               |
| Camera Module | Sensor and Serializer Module | ISX031 + MAX96717F   |

The complete camera module is:

```text
SG3S-ISX031C-GMSL2F = ISX031 Image Sensor + MAX96717F Serializer
```

---

## <b> 7-bit and 8-bit I²C Address Representation </b> 

Linux I²C tools, including `i2cdetect`, `i2cget`, `i2cset`, and `i2ctransfer`, use **7-bit I²C slave addresses**.

For example, the MAX96724 Linux I²C address is:

```text
7-bit address: 0x27
```

Some datasheets describe the I²C address as an 8-bit value that includes the read/write bit.

The corresponding 8-bit addresses are:

```text
Write address = 0x27 << 1 = 0x4E
Read address  = 0x4E | 1  = 0x4F
```

Therefore:

| Address Representation |  Value |
| ---------------------- | -----: |
| Linux 7-bit address    | `0x27` |
| 8-bit write address    | `0x4E` |

When using Linux I²C tools, always use:

```text
0x27
```

Do not use `0x4E` as the slave address in `i2ctransfer`.

---

## <b> Serializer Address Assignment </b>

The MAX96717F serializer initially uses the default 7-bit I²C address:

```text
0x40
```

During driver initialization, the GMSL driver assigns a unique address to each serializer according to its GMSL link:

| GMSL Link | Assigned Serializer Address |
| --------- | --------------------------: |
| Link 0    |                      `0x41` |
| Link 1    |                      `0x42` |
| Link 2    |                      `0x43` |
| Link 3    |                      `0x44` |

This allows multiple serializers with the same factory-default address to operate behind one MAX96724 deserializer.

For example, after driver initialization, the serializer connected to Link 2 can be accessed with:

```bash
sudo i2ctransfer -y -f 5 w2@0x43 0x02 0xbe r1
```

The address assignment is implemented in:

```text
mipi-upboard/upmipi/drivers/media/i2c/gmsl/max_des.c
```

Search for the following function:

```text
max_des_ser_new_addr_8bit
```

If addresses `0x41` through `0x44` conflict with other I²C devices, modify the serializer address assignment carefully and rebuild the driver.

> **Important:**
> The new addresses must not conflict with any other device on the same I²C bus.

---

## <b> Supported GMSL Hardware </b>

The current GMSL driver is designed for the following hardware combination:

```text
MAX96724 / MAX96724F Deserializer
            +
SG3S-ISX031C-GMSL2F Camera Module
            +
    AAEON UP Board (PHY Mapping)
```

The driver currently targets:

* MAX96724 / MAX96724F
* MAX96717F
* ISX031
* SG3S-ISX031C-GMSL2F
* Supported UP Board platforms

Using another deserializer, serializer, image sensor, or camera module may require:

* Register initialization changes
* Link configuration changes
* I²C alias configuration changes
* GPIO and reset-sequence changes
* CSI-2 data-type changes
* MIPI lane configuration changes
* V4L2 sub-device changes
* ACPI configuration changes
* Driver redevelopment

---

## <b> GMSL Driver Operations </b>

### <b> 1. Load the Complete GMSL Driver Stack </b>

Load the MAX96724 driver:

```bash
sudo modprobe max96724_driver
```

The dependent GMSL library module should be loaded automatically.

Verify the loaded modules:

```bash
lsmod | grep -E "max96724_driver|max_gmsl_lib"
```

Check the driver log:

```bash
sudo dmesg -w | grep -E "isx|max9|ipu|i2c|imx"
```

---

### <b> 2. Unload the GMSL Driver Stack </b>

Unload the driver and its dependent library:

```bash
sudo modprobe -r max96724_driver max_gmsl_lib
```

Alternatively:

```bash
sudo rmmod max96724_driver
sudo rmmod max_gmsl_lib
```

Using `modprobe -r` is recommended because it handles module dependencies more safely.

Verify that the modules have been removed:

```bash
lsmod | grep -E "max96724_driver|max_gmsl_lib"
```

No output means that the modules are no longer loaded.

> **Important:**
> Stop all active camera streams before unloading the driver.

---

### <b> 3. Bind the Driver to ACPI Node 0 </b>

Bind the MAX96724 driver to the ACPI-created I²C device:

```bash
echo "i2c-MAX96724:00" | \
  sudo tee /sys/bus/i2c/drivers/max96724/bind
```

Verify the binding:

```bash
ls -l /sys/bus/i2c/drivers/max96724/
```

A symbolic link for `i2c-MAX96724:00` should appear in the driver directory.

---

### <b> 4. Unbind the Driver from ACPI Node 0 </b>

Unbind the MAX96724 driver from the ACPI-created I²C device:

```bash
echo "i2c-MAX96724:00" | \
  sudo tee /sys/bus/i2c/drivers/max96724/unbind
```

Verify that the device entry has been removed:

```bash
ls -l /sys/bus/i2c/drivers/max96724/
```

> **Note:**
> The ACPI device name may differ between BIOS versions or platforms.

---

### <b> 5. Rescan GMSL Camera Links </b>

Trigger the GMSL driver to rescan all camera links:

```bash
echo 1 | \
  sudo tee /sys/module/max_gmsl_lib/parameters/hotplug_rescan
```

This operation can be used after:

* Connecting a GMSL camera module
* Reconnecting a camera cable
* Recovering a failed GMSL link
* Resetting the serializer or deserializer
* Testing camera hot-plug behavior

---
 
## <b> Typical Initialization and Verification Flow </b>

```bash
# 1. Load the GMSL driver
sudo modprobe max96724_driver

# 2. Verify loaded modules
lsmod | grep -E "max96724_driver|max_gmsl_lib"

# 3. Trigger a GMSL link rescan
echo 1 | \
  sudo tee /sys/module/max_gmsl_lib/parameters/hotplug_rescan

# 4. Check kernel messages
sudo dmesg | grep -Ei "gmsl|max96724|max96717|isx031"

# 5. List the detected video devices
v4l2-ctl --list-devices

# 6. Display the media topology
sudo media-ctl -p
```

---
## <b> Lock the Kernel Version </b>

The IPU, MIPI, and GMSL drivers are built for a specific Linux kernel version. An automatic kernel update may cause the existing camera kernel modules to become unavailable or incompatible.

After confirming that the GMSL camera works correctly, lock the kernel packages to prevent automatic kernel updates.

### <b> 1. Check the Current Kernel Version </b>

```bash
uname -r
```

Record this version as the validated kernel version.

### <b> 2. Lock the Kernel Packages </b>

For a standard Ubuntu generic kernel, run:

```bash
sudo apt-mark hold \
  linux-generic \
  linux-image-generic \
  linux-headers-generic
```

For an Ubuntu 24.04 HWE kernel, use the HWE meta-packages instead:

```bash
sudo apt-mark hold \
  linux-generic-hwe-24.04 \
  linux-image-generic-hwe-24.04 \
  linux-headers-generic-hwe-24.04
```

Only use the command that matches the kernel packages installed on the system.

### <b> 3. Verify the Hold Status </b>

```bash
apt-mark showhold
```

The locked kernel packages should appear in the output.

After every reboot, verify that the system is still running the validated kernel:

```bash
uname -r
```

> **Important:**
> If the expected GMSL or MIPI camera messages suddenly disappear from `dmesg`, first check the current kernel version:
>
> ```bash
> uname -r
> ```
>
> If the version differs from the validated kernel version, the system was likely upgraded automatically and booted into a kernel for which the IPU, MIPI, or GMSL modules have not been built.

### <b> Unlock the Kernel </b>

When a kernel upgrade is required, remove the hold before updating.

For the standard generic kernel:

```bash
sudo apt-mark unhold \
  linux-generic \
  linux-image-generic \
  linux-headers-generic
```

For the Ubuntu 24.04 HWE kernel:

```bash
sudo apt-mark unhold \
  linux-generic-hwe-24.04 \
  linux-image-generic-hwe-24.04 \
  linux-headers-generic-hwe-24.04
```

After upgrading the kernel, rebuild and reinstall the IPU, MIPI, and GMSL drivers for the new kernel version.

---

## <b> Important Notes </b>
 
1. Linux I²C utilities use **7-bit I²C addresses**.

2. The MAX96724 address `0x27` corresponds to:

   * `0x4E` for an 8-bit write address

3. The MAX96717F serializer uses `0x40` as its default address before driver initialization.

4. The GMSL driver reassigns serializers to `0x41`, `0x42`, `0x43`, and `0x44` for Links 0 through 3.

5. Manual I²C access with the `-f` option may conflict with an active kernel driver.

6. Stop camera streaming before unloading, rebinding, or manually reconfiguring the GMSL devices.

7. The ACPI node name may differ depending on the BIOS version and platform.

8. Different GMSL camera modules may require driver, ACPI, CSI-2, and initialization-sequence modifications.

9. Always inspect `dmesg` after loading, unloading, binding, unbinding, or rescanning the driver.

10. Verify that the BIOS camera configuration matches the physical CSI connector and camera module.
