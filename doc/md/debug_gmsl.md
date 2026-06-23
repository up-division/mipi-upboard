# <b> How to Debug GMSL </b>

When a GMSL camera cannot be detected or streamed, check the system in the following order:

1. Verify the I²C bus and device addresses.
2. Verify the V4L2 media-controller topology.
3. Check whether the GMSL kernel modules are loaded.
4. Inspect the kernel messages.
5. Rescan the GMSL links.
6. Verify the video nodes and streaming configuration.

---

### <b> 1. Check the I²C Bus </b>

List all available I²C adapters:

```bash
i2cdetect -l
```

Example MTL/ARL output:

```text
i2c-0   i2c   Synopsys DesignWare I2C adapter
i2c-5   i2c   Synopsys DesignWare I2C adapter
```

Scan the I²C bus connected to the GMSL deserializer:

```bash
sudo i2cdetect -y -r <bus>
```

For example:

```bash
sudo i2cdetect -y -r 0
```

or:

```bash
sudo i2cdetect -y -r 5
```

The correct bus depends on the UP Board platform and the physical camera connector.

#### <b> Expected I²C Addresses </b>

The following addresses may appear after the GMSL driver has been initialized:

|        Address | Device                               | Expected Behavior                                                                        |
| -------------: | ------------------------------------ | ---------------------------------------------------------------------------------------- |
| `0x27` or `UU` | MAX96724 / MAX96724F deserializer    | `UU` means that the address is currently claimed by a Linux kernel driver                |
|         `0x40` | Default MAX96717F serializer address | Used before the driver assigns a unique address to the serializer                        |
|         `0x41` | Serializer on GMSL Link 0            | Expected after Link 0 is detected and initialized                                        |
|         `0x42` | Serializer on GMSL Link 1            | Expected after Link 1 is detected and initialized                                        |
|         `0x43` | Serializer on GMSL Link 2            | Expected after Link 2 is detected and initialized                                        |
|         `0x44` | Serializer on GMSL Link 3            | Expected after Link 3 is detected and initialized                                        |
|         `0x1A` | ISX031 image sensor                  | May become visible after the camera link and sensor I²C translation have been configured |

Example scan result:

```text
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- 1a -- -- -- -- --
20: -- -- -- -- -- -- -- UU -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- 41 42 43 44 -- -- -- -- -- -- -- -- -- -- --
```

#### <b> Address Interpretation </b>

##### <b> MAX96724 Deserializer Address </b>

```text
0x27
```

If the output shows:

```text
UU
```

the deserializer is detected, but the address is occupied by the kernel driver.

This is normally expected after the MAX96724 driver has been loaded.

##### <b> MAX96717F Default Address </b>

```text
0x40
```

Each MAX96717F serializer uses `0x40` as its default address before initialization.

Because multiple serializers may use the same default address, the driver isolates each GMSL link and reassigns the serializers to unique addresses.

##### <b> Reassigned Serializer Addresses </b>

```text
Link 0 -> 0x41
Link 1 -> 0x42
Link 2 -> 0x43
Link 3 -> 0x44
```

For example, if a camera module is connected to Link 0, address `0x41` should appear after the driver initializes the link or after a hot-plug rescan.

If `0x41` does not appear, possible causes include:

* The GMSL cable is disconnected or damaged.
* The camera is connected to a different link.
* The serializer is not responding at its default address.
* The deserializer link is not locked.
* The serializer address reassignment failed.
* The driver did not complete initialization.

##### <b> ISX031 Sensor Address </b>

```text
0x1A
```

The ISX031 sensor is located behind the MAX96717F serializer.

It may not be directly visible until the following operations are completed:

* The GMSL link is locked.
* The serializer has been initialized.
* The I²C translation or alias has been configured.
* The sensor has been powered and released from reset.

With the current driver implementation, `0x1A` may appear only after the camera is initialized or streaming. Therefore, the absence of `0x1A` while the camera is idle does not always indicate a hardware failure.

---

### <b> 2. Check the V4L2 Media Topology </b>

List all media-controller devices:

```bash
ls -l /dev/media*
```

Display the complete topology of `/dev/media0`:

```bash
sudo media-ctl -p -d /dev/media0
```

To display only the MAX96724 and Intel IPU6 CSI-2 entities:

```bash
sudo media-ctl -p -d /dev/media0 | \
awk -v RS= '/- entity [0-9]+: (Intel IPU6 CSI2 0|Intel IPU6 CSI2 1|Intel IPU6 CSI2 2|Intel IPU6 CSI2 4|max967)/'
```

#### <b> Expected Topology </b>

A correct topology should contain a MAX96724 V4L2 sub-device connected to an Intel IPU6 CSI-2 receiver.

The exact entity names and pad numbers may vary depending on the platform, BIOS configuration, CSI port, and driver version.

A simplified expected connection is:

```text
MAX96724 / MAX96724F Deserializer
        |
        | MIPI CSI-2
        v
Intel IPU6 CSI2 Receiver
        |
        v
Intel IPU6 ISYS Capture Pipeline
        |
        v
/dev/videoX
```

A simplified `media-ctl` topology may look similar to:

```text
- entity: max96724
    type V4L2 subdev
    source pad -> Intel IPU6 CSI2 sink pad

- entity: Intel IPU6 CSI2 0
    type V4L2 subdev
    sink pad <- max96724 source pad
    source pad -> Intel IPU6 ISYS capture entity
```

Important items to verify:

* A `max96724` entity exists.
* The MAX96724 entity has a source pad.
* The source pad is linked to the correct Intel IPU6 CSI-2 entity.
* The media link is enabled.
* The CSI-2 receiver corresponds to the BIOS-configured physical port.
* A capture video node is present downstream.

An enabled media link is normally displayed with:

```text
[ENABLED]
```

For example:

```text
pad0: Source
    -> "Intel IPU6 CSI2 0":0 [ENABLED]
```

#### <b> If the MAX96724 Entity Is Missing </b>

Possible causes include:

* The MAX96724 driver is not loaded.
* The ACPI camera node is missing or disabled.
* The BIOS camera configuration is incorrect.
* The driver failed during probe.
* The kernel version does not match the installed driver module.
* The driver is bound to a different ACPI node.
* The camera is attached to another CSI port.

Check:

```bash
lsmod | grep -E "max96724|max_gmsl"
sudo dmesg -w | grep -E "isx|max9|ipu|i2c|imx"
```

#### <b> If the MAX96724 Entity Exists but Is Not Linked </b>

Possible causes include:

* Incorrect BIOS CSI port selection.
* A mismatch between the platform configuration and the physical connector.

---

### <b> 3. Check the GMSL Kernel Modules </b>

Check whether the required modules are loaded:

```bash
lsmod | grep -E "max96724_driver|max_gmsl_lib"
```

Expected output should contain modules similar to:

```text
max96724_driver
max_gmsl_lib
```

Load the GMSL driver if it is not present:

```bash
sudo modprobe max96724_driver
```

Check again:

```bash
lsmod | grep -E "max96724_driver|max_gmsl_lib"
```

Check whether the module exists for the currently running kernel:

```bash
modinfo max96724_driver
modinfo max_gmsl_lib
```

If `modinfo` reports:

```text
modinfo: ERROR: Module max96724_driver not found
```

the module may not have been installed for the currently running kernel.

Compare the current kernel version with the module version:

```bash
uname -r
modinfo -F vermagic max96724_driver
```

The beginning of the `vermagic` output should match the output of `uname -r`.

Example:

```text
uname -r:
6.17.0-xx-generic

modinfo -F vermagic max96724_driver:
6.17.0-xx-generic SMP preempt mod_unload modversions
```

If the versions do not match, rebuild and reinstall the GMSL drivers for the current kernel.

> **Important:**
> If the GMSL or MIPI camera messages suddenly disappear from `dmesg`, check `uname -r` first. The system may have automatically booted into a newer kernel for which the camera modules have not been built.

---

### <b> 4. Read and Interpret the Kernel Messages </b>

Monitor the GMSL, image sensor, Intel IPU, CSI-2, PHY, and I²C messages in real time:

```bash
sudo dmesg -w | grep -Ei "isx|max9|gmsl|ipu|i2c|imx"
```

To inspect messages that have already been generated:

```bash
sudo dmesg | grep -Ei "isx|max9|gmsl|ipu|i2c|imx"
```

GMSL problems can generally be divided into the following conditions:

1. MAX96724 probe fails with `-121`.
2. Streaming returns `-5`.
3. Streaming starts but nothing happens.
4. The Intel IPU continuously reports errors.
5. The image is corrupted or contains many abnormal colored pixels.
6. A previous streaming application was not closed correctly.

---

#### <b> 4.1 MAX96724 Probe Failure: `ret = -121` </b>

During the MAX96724 driver probe, the driver performs the following operations:

```text
Power on the MAX96724
        |
        v
Initialize the required MAX96724 registers
        |
        v
Detect GMSL Links 0 through 3
        |
        v
Remap the serializer I²C address on each detected link
        |
        v
Register the V4L2 sub-device and media topology
```

If the driver cannot access a MAX96724 register during probe, the kernel log may show:

```text
MAX96724 register write failed, ret = -121
```

Return value `-121` corresponds to:

```text
-EREMOTEIO
Remote I/O error
```

This means that the I²C controller attempted to communicate with the MAX96724, but the deserializer did not acknowledge the transaction.


Common causes include:

1. The MAX96724 deserializer is not connected.
2. The BIOS CSI port or I²C bus is configured incorrectly.
3. The UP Board platform is not defined in `mipi-upboard.h`.
4. The GPIO line number in `mipi-upboard.h` is incorrect.
5. The MAX96724 power-enable GPIO does not output logic high.
6. The deserializer does not receive the required power supply.
7. The driver is accessing the wrong I²C bus.

Check whether the MAX96724 appears on the expected I²C bus:

```bash
sudo i2cdetect -y -r <bus>
```

The MAX96724 should normally appear as:

```text
0x27
```

or:

```text
UU
```

`UU` means that the address is currently claimed by a kernel driver.

If neither `0x27` nor `UU` appears, verify:

```text
1. MAX96724 hardware connection
2. BIOS CSI port configuration
3. I²C bus selection
4. Board definition in mipi-upboard.h
5. MAX96724 power-enable GPIO line number
6. Actual GPIO output level
7. MAX96724 power supply
```

If the power-enable GPIO remains low, all subsequent MAX96724 register accesses may return `-121`.

---

#### <b>4.2 Streaming Returns `ret = -5`</b>

If the application attempts to stream from an unavailable or disabled video node, the stream operation may return:

```text
ret = -5
```

Return value `-5` corresponds to:

```text
-EIO
Input/output error
```

In the current driver implementation, `-5` usually means that one of the following conditions occurred:

1. The software camera mask of the selected video node is `0`.
2. No camera was detected on the GMSL link corresponding to the video node.
3. The selected video node or its media path has not been enabled.
4. A previous streaming process did not close correctly and left the video node in an invalid state.


#### <b> Camera Mask Is `0` </b>

The GMSL driver updates the software camera mask during the initial link scan or hot-plug rescan.

If no camera is detected on a link, the mask corresponding to that video node is set to `0`.

The expected sequence is:

```text
Initial scan or hot-plug rescan
        |
        v
No camera is detected on the GMSL link
        |
        v
The corresponding video-node mask is set to 0
        |
        v
The application starts streaming from that video node
        |
        v
STREAMON returns -5
```

Therefore, in practical use, `-5` usually occurs when the application attempts to stream from a video node whose corresponding camera is not connected or was not detected.

Common causes include:

* No camera is connected to the selected GMSL link.
* The camera is connected to another link.
* The GMSL cable is disconnected or damaged.
* The serializer was not detected during the scan.
* The GMSL link is not locked.
* The wrong `/dev/videoX` node was selected.
* The camera was connected or reconnected without running a hot-plug rescan.

Check whether the serializer corresponding to the selected link appears on the I²C bus:

```bash
sudo i2cdetect -y -r <bus>
```

If the serializer address is missing, the camera on that link was not detected correctly.

Trigger a GMSL link rescan when necessary:

```bash
echo 1 | \
sudo tee /sys/module/max_gmsl_lib/parameters/hotplug_rescan
```

After the rescan, check the I²C bus again and retry the stream.

#### <b> Video Node or Media Path Is Not Enabled </b>

Even if the camera is detected, `-5` may occur when the selected video node or its media path has not been enabled.

Display the media-controller topology:

```bash
sudo media-ctl -p -d /dev/media0
```

Locate the entity associated with the selected `/dev/videoX` node and verify that its upstream path is enabled.

For example, when streaming from `/dev/video32`, the topology should contain the corresponding capture entity and an enabled upstream connection similar to:

```text
- entity: Intel IPU6 ISYS Capture 32
    device node name /dev/video32

    <- "Intel IPU6 CSI2 4":1 [ENABLED]
```

The connection from the MAX96724 to the Intel IPU6 CSI-2 receiver should also be enabled:

```text
"max96724":1 -> "Intel IPU6 CSI2 4":0 [ENABLED]
```

The complete active path should be similar to:

```text
MAX96724
    |
    | [ENABLED]
    v
Intel IPU6 CSI2 4
    |
    | [ENABLED]
    v
Intel IPU6 ISYS Capture 32
    |
    v
/dev/video32
```

If `/dev/video32` exists but the upstream media path is not enabled, the node cannot receive frames and `STREAMON` may return `-5`.

Typical CSI port and video-node mappings are:

| Platform                 | CSI Port | Video Nodes                   |
| ------------------------ | -------: | ----------------------------- |
| Meteor Lake / Arrow Lake |    CSI 0 | `/dev/video0`–`/dev/video3`   |
| Meteor Lake / Arrow Lake |    CSI 4 | `/dev/video32`–`/dev/video35` |
| Raptor Lake / Alder Lake |    CSI 1 | `/dev/video8`–`/dev/video11`  |
| Raptor Lake / Alder Lake |    CSI 2 | `/dev/video16`–`/dev/video19` |

Before streaming, verify that:

* The selected video node belongs to the correct CSI port.
* The video node corresponds to the physically connected GMSL link.
* The software camera mask for the node is enabled.
* Every required upstream media link is marked as `[ENABLED]`.

---

#### <b> 4.3 Streaming Starts but Nothing Happens </b>

In this condition:

* The stream command does not return an immediate error.
* No image is displayed.
* No frame is returned to the application.
* No obvious GMSL error appears.
* No Intel IPU error appears.
* The application may remain blocked while waiting for a frame.

This means that the software-side stream request was accepted, but the Intel IPU never received a complete CSI-2 frame.

The expected streaming path is:

```text
Application issues V4L2 STREAMON
        |
        v
V4L2 calls max_des.c max_des_enable_streams()
        |
        v
The corresponding link is enabled in MAX96724 register 0x0006
        |
        v
Serializer register 0x02BE is written with 0x10
        |
        v
The camera starts transmitting frames
        |
        v
MAX96724 forwards CSI-2 data to the Intel IPU
        |
        v
The IPU receives and completes a frame
```

If one stage in this path does not complete correctly, the IPU may remain active and continue waiting without producing an immediate error.

Possible causes include:

1. The corresponding GMSL link is not enabled in MAX96724 register `0x0006`.
2. The V4L2 stream command does not reach `max_des.c` correctly.
3. `max_des_enable_streams` is not executed.
4. The video-node-to-link mapping is incorrect.
5. Serializer register `0x02BE` is not changed to `0x10`.
6. The camera does not receive the stream command.
7. The camera remains in standby.
8. The image sensor does not produce valid frame data.
9. The MAX96724 does not forward the selected link through the correct CSI-2 PHY output.
10. The link-to-pipe mapping, virtual channel, data type, or PHY output routing is incorrect.
11. **The redesigned deserializer board uses a different MIPI PHY output**

    The original MAX96724 board uses PHY 1 for the MIPI CSI-2 output. If a redesigned board routes the signal through another PHY, the PHY-related settings in `max96724.c` must be updated to match the new PCB routing. Otherwise, streaming may start successfully, but no complete frame reaches the Intel IPU.


Read MAX96724 register `0x0006`:

```bash
sudo i2ctransfer -y -f <bus> \
  w2@0x27 0x00 0x06 r1
```

Verify that the bit corresponding to the selected link is enabled.

Read serializer register `0x02BE`:

```bash
sudo i2ctransfer -y -f <bus> \
  w2@<serializer_addr> 0x02 0xbe r1
```

The expected value during streaming is:

```text
0x10
```

The main difference between `-5` and no-response behavior is:

| Symptom                           | Interpretation                                             |
| --------------------------------- | ---------------------------------------------------------- |
| `STREAMON` returns `-5`           | The selected video-node mask is `0`                        |
| Stream starts but nothing happens | The node is enabled, but no complete frame reaches the IPU |

> **Note:**
> The absence of a kernel error does not mean that the complete stream path is working. The IPU may still be waiting for a frame that never arrives.

---

#### <b> 4.4 Intel IPU Continuously Reports Errors </b>

In this condition:

* Streaming starts successfully.
* The Intel IPU receives CSI-2 activity.
* The kernel log continuously prints IPU, CSI-2, PHY, CRC, ECC, or frame-related errors.
* No stable image is produced.

Unlike the no-response condition, continuous IPU errors normally mean that data reaches the IPU, but the received signal or configuration is invalid.

The failure sequence is:

```text
Camera starts transmitting
        |
        v
MAX96724 outputs CSI-2 data
        |
        v
The Intel IPU receives CSI-2 activity
        |
        v
Lane count, timing, frequency, deskew, or PHY mapping is incorrect
        |
        v
The IPU continuously reports errors
```

The most common causes are:

1. Incorrect number of MIPI CSI-2 data lanes.
2. Incorrect deskew configuration.
3. Incorrect `link_freq`.
4. Incorrect PHY mapping.

---

#### <b> 4.4.1 Incorrect Number of MIPI CSI-2 Data Lanes </b>

The number of MIPI CSI-2 data lanes must be consistent across the BIOS configuration, MAX96724 configuration, ACPI endpoint, V4L2 driver, and Intel IPU receiver.

For the current GMSL configuration, the BIOS camera interface must be configured for:

```text
4 MIPI CSI-2 data lanes
```

The MAX96724 lane mode configured through register `0x08A0` must also match the four-lane output configuration.

If the BIOS is configured with an incorrect lane count, or if the lane mode in register `0x08A0` is configured incorrectly, the Intel IPU may receive CSI-2 activity but fail to reconstruct valid frames.

Possible symptoms include:

* Continuous Intel IPU or CSI-2 error messages
* PHY synchronization errors
* CRC or ECC errors
* Incomplete frames
* No valid image output
* Corrupted or unstable images
* Streaming starts, but frames cannot be completed


Verify the following settings:

1. The BIOS camera interface is configured for four MIPI CSI-2 lanes.
2. The ACPI endpoint reports the correct number of data lanes.
3. The V4L2 sub-device reports the correct lane configuration.
4. MAX96724 register `0x08A0` uses the correct lane mode.
5. The MAX96724 PHY and lane mapping match the physical board routing.
6. The Intel IPU CSI-2 receiver is configured for the same lane count.

> **Important:**
> The BIOS lane count and the MAX96724 lane mode must match. A valid media topology does not guarantee that the lane configuration is correct. A lane-count mismatch commonly causes continuous IPU errors after streaming starts.


#### <b> 4.4.2 Deskew Configuration </b>

**Skew** is the timing difference between MIPI CSI-2 data lanes. Ideally, all lanes should arrive at the receiver at nearly the same time. Cable length, PCB routing, signal quality, and high lane rates may cause one lane to arrive earlier or later than the others.

**Deskew** compensates for this lane-to-lane timing difference so that the receiver can correctly synchronize the CSI-2 data.

According to the MIPI D-PHY specification, deskew calibration is required when the per-lane data rate is greater than `1.5 Gbps`. Because MIPI D-PHY transfers data on both clock edges, the lane data rate is twice the V4L2 `link_freq` value. Therefore, a lane rate of `1.5 Gbps` corresponds to a `link_freq` of `750 MHz`.

```text
Lane data rate = link_freq × 2

750 MHz × 2 = 1.5 Gbps
```

Deskew must be enabled on both sides of the MIPI connection:

* MAX96724 CSI-2 transmitter
* Intel IPU CSI-2 receiver

The Intel IPU driver should normally enable deskew automatically when `link_freq` is greater than `750 MHz`. The MAX96724 deskew configuration must also be enabled for the active PHY.

The following MAX96724 registers can be used to inspect or adjust the deskew settings:

| MAX96724 PHY | Deskew Registers   |
| ------------ | ------------------ |
| PHY 0        | `0x0903`, `0x0904` |
| PHY 1        | `0x0943`, `0x0944` |
| PHY 2        | `0x0983`, `0x0984` |
| PHY 3        | `0x09C3`, `0x09C4` |

For example, read the PHY 1 deskew registers:

```bash
sudo i2ctransfer -y -f <bus> w2@0x27 0x09 0x43 r1
sudo i2ctransfer -y -f <bus> w2@0x27 0x09 0x44 r1
```

These registers can also be modified manually for debugging:

```bash
sudo i2ctransfer -y -f <bus> \
  w3@0x27 <reg_high> <reg_low> <value>
```

Possible symptoms of an incorrect deskew configuration include:

* Continuous CSI-2 PHY errors
* CRC or ECC errors
* Frame-start or frame-end errors
* Incomplete frames
* Corrupted images
* Abnormal colored pixels
* No valid completed frames

Possible causes include:

* Deskew is enabled only on the MAX96724 or only on the Intel IPU.
* The Intel IPU and MAX96724 deskew settings do not match.
* The wrong MAX96724 PHY deskew registers are configured.
* The configured `link_freq` does not match the actual lane rate.
* Cable length or signal quality causes excessive lane-to-lane skew.

Verify that the deskew configuration matches the active MAX96724 PHY, MIPI lane rate, number of data lanes, Intel IPU receiver settings, and physical board routing.

---

#### <b> 4.4.3 Incorrect `link_freq` </b> 

The `link_freq` value represents the MIPI CSI-2 link frequency expected by the Linux camera pipeline.

If the configured value does not match the actual MAX96724 CSI-2 output frequency, the IPU may use incorrect timing parameters.

Possible symptoms include:

* Continuous IPU or CSI-2 errors
* CRC or ECC errors
* Frame synchronization failures
* No valid image
* Unstable or corrupted output

The configured `link_freq` must be consistent with:

* Camera resolution
* Frame rate
* Pixel format
* Bit depth
* Number of MIPI lanes
* MAX96724 CSI-2 output rate
* IPU6 receiver configuration

A mismatch may occur if:

* The camera mode was changed without updating `link_freq`.
* The lane count was changed.
* The driver uses a fixed value that does not match the selected mode.
* The MAX96724 output rate differs from the value reported by the V4L2 sub-device.

---

#### <b> 4.4.4 Incorrect PHY Mapping </b>

PHY mapping defines how the MAX96724 CSI-2 output is connected to the Intel IPU6 CSI-2 receiver.

If the mapping does not match the physical board routing or BIOS configuration, the IPU may receive data from the wrong CSI port or lane group.

Possible symptoms include:

* Continuous CSI-2 receiver errors
* No valid frame completion
* Incorrect lane synchronization
* Data appears on the wrong CSI receiver
* Media topology exists, but streaming fails
* The same driver works on one platform but fails on another

Possible causes include:

* Incorrect physical CSI connector
* Incorrect BIOS CSI port
* Incorrect IPU6 CSI receiver number
* Incorrect ACPI endpoint
* Incorrect lane mapping or lane order
* Incorrect board definition in `mipi-upboard.h`

Verify:

```text
1. UP Board platform
2. Physical CSI connector
3. BIOS CSI port configuration
4. Intel IPU6 CSI-2 receiver number
5. MIPI lane mapping
```

---

#### <b> 4.5 Corrupted Image or Abnormal Colored Pixels </b>

In this condition, streaming may start and an image may be displayed, but the image contains:

* Large numbers of white pixels
* Pink pixels
* Blue pixels
* Green pixels
* Random colored dots
* Incorrect image colors
* Partially corrupted frames
* Intermittent image breakup

The Intel IPU usually also prints continuous CSI-2 or PHY errors.

A common cause is that the cable between the MAX96724 deserializer and the Intel IPU is too long.

This cable carries the MIPI CSI-2 signal. A configuration that works correctly with a short cable may become unstable when a longer cable is used.

Possible electrical causes include:

* Signal attenuation
* Lane-to-lane timing skew
* Increased jitter
* Signal reflection
* Crosstalk
* Cable impedance mismatch
* Poor connector contact
* Reduced PHY timing margin

The most reliable solution is:

> Replace the DES-to-IPU cable with a shorter cable.

A shorter cable normally provides:

* Better signal integrity
* Lower attenuation
* Lower lane skew
* Fewer CSI-2 errors
* More stable colors
* Fewer corrupted pixels

---

#### <b> 4.5.1 Advanced DWC PHY Tuning </b>

If the long cable must be used, the following DWC PHY parameters may be adjusted:

```bash
echo -1 | sudo tee \
  /sys/module/intel_ipu6_isys/parameters/dwc_mbps_override

echo -1 | sudo tee \
  /sys/module/intel_ipu6_isys/parameters/dwc_hsfreq_index_override

echo -1 | sudo tee \
  /sys/module/intel_ipu6_isys/parameters/dwc_osc_freq_target_override

echo -1 | sudo tee \
  /sys/module/intel_ipu6_isys/parameters/dwc_deskew_polarity_rw_override
```

The implementation and parameter tables are located in:

```text
upmipi/drivers/media/pci/intel/ipu6/ipu6-isys-dwc-phy.c
```

Inspect the relevant parameter definitions and lookup tables:

```bash
grep -nE \
"dwc_mbps_override|dwc_hsfreq_index_override|dwc_osc_freq_target_override|dwc_deskew_polarity_rw_override" \
upmipi/ipu6/drivers/media/pci/intel/ipu6/ipu6-isys-dwc-phy.c
```

Read the current values:

```bash
cat /sys/module/intel_ipu6_isys/parameters/dwc_mbps_override
cat /sys/module/intel_ipu6_isys/parameters/dwc_hsfreq_index_override
cat /sys/module/intel_ipu6_isys/parameters/dwc_osc_freq_target_override
cat /sys/module/intel_ipu6_isys/parameters/dwc_deskew_polarity_rw_override
```

These parameters are difficult to tune because the result depends on:

* Cable length
* Cable quality
* Platform
* Board revision
* MIPI lane rate
* Lane count
* Resolution
* Frame rate
* Pixel format
* MAX96724 output configuration


> **Important:**
>
>These override parameters were added by SPENK for the DWC PHY path used by platforms such as Meteor Lake and Arrow Lake. In the original Intel IPU6 driver, these PHY settings are fixed in the source code and cannot be adjusted dynamically through userspace commands.
>
> They do not apply to the MCD or JSL PHY paths. For MCD or JSL platforms, replacing the cable with a shorter cable is the recommended solution.

PHY tuning is only a workaround. The fundamental and most reliable solution is to shorten the cable between the MAX96724 and the Intel IPU.

---

#### <b> 4.6 Previous Streaming Process Was Not Closed Correctly </b>

A previous streaming application may remain active if it was not terminated correctly.

Examples include:

* The application crashed.
* The terminal was closed while streaming.
* A GStreamer process remains in the background.
* A Python or V4L2 application is still holding the video node.
* The stream shutdown sequence did not complete.

This condition may produce symptoms similar to:

* `ret = -5`
* Streaming starts but no image appears
* The application waits indefinitely
* The video device is busy
* No clear GMSL error appears

Before debugging the hardware or driver, check whether another process is still using a video node:

```bash
sudo fuser -v /dev/video*
```

Example output:

```text
                     USER        PID ACCESS COMMAND
/dev/video0:         user       2451 F.... gst-launch-1.0
```

Inspect the process:

```bash
ps -fp <PID>
```

Terminate it normally:

```bash
sudo kill <PID>
```

If it does not stop:

```bash
sudo kill -9 <PID>
```

To terminate every process using a specific video node:

```bash
sudo fuser -k /dev/video0
```

Confirm that no process remains:

```bash
sudo fuser -v /dev/video*
```

If the IPU or driver remains in an invalid state, reload the GMSL driver:

```bash
sudo modprobe -r max96724_driver max_gmsl_lib
sudo modprobe max96724_driver
```

Then trigger a rescan:

```bash
echo 1 | \
sudo tee /sys/module/max_gmsl_lib/parameters/hotplug_rescan
```

If the system still cannot recover, reboot:

```bash
sudo reboot
```

---

#### <b> 4.7 Error Summary </b>

| Stage               | Symptom                                                 | Primary Interpretation                                                                              |
| ------------------- | ------------------------------------------------------- | --------------------------------------------------------------------------------------------------- |
| Driver probe        | Register access returns `-121`                          | MAX96724 does not acknowledge the local I²C transaction                                             |
| Stream start        | `STREAMON` returns `-5`                                 | The selected video-node mask is `0`; normally no camera was detected on that link                   |
| After stream start  | No image, no message, and no IPU error                  | The IPU is waiting because no complete frame reaches the capture pipeline                           |
| After stream start  | IPU continuously reports errors                         | CSI-2 activity reaches the IPU, but deskew, `link_freq`, PHY mapping, or signal timing is incorrect |
| During streaming    | Image contains white, pink, blue, or green noise points | The DES-to-IPU cable or PHY configuration causes CSI-2 signal corruption                            |
| Before a new stream | Behavior resembles `-5` or no response                  | A previous process may still be holding the video node                                              |

---

#### <b> 4.8 Recommended Troubleshooting Order </b>

```text
1. Check whether another process is using /dev/video*
        |
        v
2. Check the MAX96724 probe result
        |
        v
3. If probe failed, investigate ret = -121
        |
        v
4. Start the stream
        |
        v
5. If STREAMON returns -5, check the video-node mask and camera detection
        |
        v
6. If nothing happens, check the complete stream-enable path
        |
        v
7. If the IPU continuously reports errors, check deskew, link_freq, and PHY mapping
        |
        v
8. If the image contains abnormal colored pixels, test with a shorter DES-to-IPU cable
```
---

### <b> 5. Verify the Driver Binding </b>

Check the MAX96724 driver directory:

```bash
ls -l /sys/bus/i2c/drivers/max96724/
```

Check all I²C device names:

```bash
grep -H . /sys/bus/i2c/devices/*/name 2>/dev/null
```

Find the MAX96724 ACPI-created device:

```bash
find /sys/bus/i2c/devices -maxdepth 1 \
  -type l \
  -printf "%f\n" | grep -i "MAX96724"
```

A typical device name is:

```text
i2c-MAX96724:00
```

Bind the device manually:

```bash
echo "i2c-MAX96724:00" | \
sudo tee /sys/bus/i2c/drivers/max96724/bind
```

Unbind the device manually:

```bash
echo "i2c-MAX96724:00" | \
sudo tee /sys/bus/i2c/drivers/max96724/unbind
```

> **Note:**
> The actual ACPI device name may differ between BIOS versions and UP Board platforms.

After binding, check:

```bash
sudo dmesg | tail -n 100
sudo media-ctl -p
v4l2-ctl --list-devices
```

---

### <b> 6. Rescan the GMSL Links </b>

Trigger a GMSL hot-plug rescan:

```bash
echo 1 | \
sudo tee /sys/module/max_gmsl_lib/parameters/hotplug_rescan
```

Monitor the kernel log during the rescan:

```bash
sudo dmesg -w | grep -Ei \
"gmsl|max96724|max96717|isx031|link|camera"
```

After rescanning, check the I²C bus again:

```bash
sudo i2cdetect -y -r <bus>
```

Expected serializer addresses:

```text
Link 0 -> 0x41
Link 1 -> 0x42
Link 2 -> 0x43
Link 3 -> 0x44
```

If a newly connected camera is detected correctly, its corresponding serializer address should appear.

---

### <b> 7. Restart the GMSL Driver </b>

Stop all active camera streams before restarting the driver.

Unload the driver:

```bash
sudo modprobe -r max96724_driver max_gmsl_lib
```

Wait briefly:

```bash
sleep 1
```

Reload the driver:

```bash
sudo modprobe max96724_driver
```

Trigger a link rescan:

```bash
echo 1 | \
sudo tee /sys/module/max_gmsl_lib/parameters/hotplug_rescan
```

Verify the result:

```bash
lsmod | grep -E "max96724_driver|max_gmsl_lib"
sudo dmesg | tail -n 100
sudo media-ctl -p
v4l2-ctl --list-devices
```

---

### <b> 8. Check the Video Devices </b>

List all video devices:

```bash
ls -l /dev/video*
```

Display the V4L2 devices:

```bash
v4l2-ctl --list-devices
```

Inspect a specific video node:

```bash
v4l2-ctl -d /dev/video0 --all
```

List the supported formats:

```bash
v4l2-ctl -d /dev/video0 --list-formats-ext
```

Replace `/dev/video0` with the actual video node connected to the GMSL camera pipeline.

If no `/dev/videoX` node is created, check:

* Whether the MAX96724 sub-device is registered.
* Whether the media links are enabled.
* Whether the IPU6 CSI-2 receiver is present.
* Whether the BIOS camera configuration is correct.
* Whether the driver probe completed successfully.

---

### <b> GMSL Troubleshooting Summary </b>

The recommended troubleshooting order is:

```text
1. Check the I²C bus
2. Check the media topology
3. Check the kernel log
4. Check the build environment and installed kernel modules
```

---

#### <b> 1. Check the I²C Bus First </b>

List the available I²C buses:

```bash
i2cdetect -l
```

Scan the I²C bus connected to the MAX96724:

```bash
sudo i2cdetect -y -r <bus>
```

Typical platform mappings are:

| Platform                 | GMSL I²C Bus |
| ------------------------ | ------------ |
| Meteor Lake / Arrow Lake | `0` or `5`   |
| Raptor Lake / Alder Lake | `1` or `5`   |

| I²C Scan Result                                                                                   | Possible Cause                                                                                                          | Recommended Check                                                                                                                                                                |
| ------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Neither `0x27` nor `UU` appears                                                                   | The MAX96724 is not powered, the DES board is not connected, or the power-enable GPIO did not output logic high         | Check the DES power supply, cable connection, GPIO line number in `mipi-upboard.h`, and actual GPIO output                                                                         |
| `0x27` appears instead of `UU`                                                                    | The MAX96724 is present, but the kernel driver is not bound to it                                                       | Check whether the driver was unloaded or unbound, whether BIOS/ACPI describes the correct I²C bus and address, and whether a kernel update caused the driver module to disappear |
| A connected link still shows serializer address `0x40`                                            | The camera was recently connected or reconnected, but the driver has not rescanned the links                            | Trigger `hotplug_rescan` and scan the bus again                                                                                                                                  |
| A camera is connected to Link 0–3, but neither `0x40` nor the corresponding `0x41`–`0x44` appears | The DES link, GMSL cable, serializer, or camera module may be faulty                                                    | Check link lock, replace the GMSL cable, and test the camera module on another link                                                                                              |
| Streaming is active but no image appears                                                          | The stream command may be incorrect, the wrong video node may be selected, or the camera may not have started streaming | Check whether sensor address `0x1A` appears and read serializer register `0x02BE`; its value should be `0x10` during streaming                                                   |

Serializer address mapping:

| GMSL Link | Serializer Address |
| --------- | -----------------: |
| Link 0    |             `0x41` |
| Link 1    |             `0x42` |
| Link 2    |             `0x43` |
| Link 3    |             `0x44` |

Trigger a link rescan:

```bash
echo 1 | \
sudo tee /sys/module/max_gmsl_lib/parameters/hotplug_rescan
```

Read serializer register `0x02BE`:

```bash
sudo i2ctransfer -y -f <bus> \
  w2@<serializer_addr> 0x02 0xbe r1
```

---

#### <b> 2. Check the Media Topology </b>

Display the media-controller topology:

```bash
sudo media-ctl -p -d /dev/media0
```

| Topology Result                                                         | Expected Behavior or Possible Cause                                                                           |
| ----------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| One DES board is connected                                              | One `max96724` entity should appear                                                                           |
| Two DES boards are connected                                            | Two `max96724` entities should appear                                                                         |
| The number of `max96724` entities does not match the connected hardware | Check the BIOS camera configuration, ACPI nodes, I²C bus mapping, driver binding, and physical DES connection |
| The expected video node is disabled                                     | Enable the software mask for the video node before streaming                                                  |
| The topology exists but the wrong node is used                          | Verify the relationship between the CSI port, `max96724` entity, and `/dev/videoX` node                       |

Typical CSI port and video-node mappings are:

| Platform                 | CSI Ports | Related Video Nodes                                            |
| ------------------------ | --------- | -------------------------------------------------------------- |
| Meteor Lake / Arrow Lake | CSI 0 / 4 | `/dev/video0`–`/dev/video3` and `/dev/video32`–`/dev/video35`  |
| Raptor Lake / Alder Lake | CSI 1 / 2 | `/dev/video8`–`/dev/video11` and `/dev/video16`–`/dev/video19` |

Before streaming, the software mask for the video node corresponding to the connected camera must be enabled.

---

#### <b> 3. Check the Kernel Log </b>

Monitor the relevant messages in real time:

```bash
sudo dmesg -w | grep -Ei "isx|max9|gmsl|ipu|i2c|imx"
```

| Symptom or Kernel Log                                                                                     | Possible Cause                                                                                     | Recommended Check                                                                                                                                |
| --------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| MAX96724 register access returns `-121`                                                                   | The MAX96724 did not acknowledge the I²C transaction                                               | Check DES power, power-enable GPIO, physical connection, BIOS/ACPI I²C bus and address, and whether `0x27` or `UU` appears                       |
| MAX96724 probe succeeds                                                                                   | The log should show the register values written during MAX96724 initialization                     | Confirm that link detection, serializer remapping, and V4L2 registration continue normally                                                       |
| Streaming returns `-5` | The selected video node is unavailable for streaming because its software mask is `0`, its media path is not enabled | Check whether the correct video node was selected, confirm that the camera was detected and the media path is marked `[ENABLED]`, run a hot-plug rescan after reconnecting the camera |                              |
| Streaming starts but no image, message, or IPU error appears                                              | The IPU is waiting because no complete frame reaches it                                            | Check `link_freq` and DPLL configuration, where `DPLL = 2 × link_freq`; also check sensor address `0x1A` and serializer register `0x02BE = 0x10` |
| The IPU continuously reports errors after streaming starts                                                | The IPU receives CSI-2 activity, but the transport or PHY configuration is incorrect               | Check lane count, deskew, link_freq, DPLL, PHY mapping, DES-to-IPU cable length, and ISYS operating frequency                                  |
| Streaming works but the image contains many white, pink, blue, or green dots, or the colors are incorrect | CSI-2 signal corruption caused by a long DES-to-IPU cable or incorrect deskew configuration        | Replace the cable with a shorter cable first; if necessary, inspect the deskew and DWC PHY parameters                                            |
| Streaming returns `-5` or has no response after a previous test                                           | A previous streaming process may still be holding the video node or may not have stopped correctly | Check active processes with `fuser`, terminate them, or reboot the system                                                                        |

Check for processes still using video devices:

```bash
sudo fuser -v /dev/video*
```

Terminate the remaining process:

```bash
sudo kill <PID>
```

If it does not terminate normally:

```bash
sudo kill -9 <PID>
```

If the driver or IPU state cannot be recovered easily:

```bash
sudo reboot
```

---

#### <b> 5. Check the Build Environment </b >

If an error occurs during `make`, driver installation, or module loading, verify the kernel and generated modules.

| Check            | Expected Result                                                                        |
| ---------------- | -------------------------------------------------------------------------------------- |
| Running kernel   | The validated kernel version should be `6.17.x`                                        |
| Kernel headers   | The installed headers must match `uname -r`                                            |
| Compiled modules | IPU, MIPI, and GMSL `.ko` files must be generated and installed for the running kernel |
| Module version   | The module `vermagic` must match the running kernel                                    |

Check the current kernel:

```bash
uname -r
```

Check the matching kernel-header directory:

```bash
ls -ld /lib/modules/$(uname -r)/build
```

Search for the installed IPU, MIPI, and GMSL modules:

```bash
find /lib/modules/$(uname -r) \
  -type f -name "*.ko*" | \
  grep -Ei "ipu|max96724|gmsl|mipi"
```

The generated modules should appear under the module directory used by the installation script, typically:

```text
/lib/modules/$(uname -r)/updates/
```

Check whether the module can be found:

```bash
modinfo max96724_driver
modinfo max_gmsl_lib
```

If the running kernel is no longer `6.17.x`, or the expected `.ko` files are missing, reinstall or rebuild the drivers for the current kernel. For the validated development environment, return to the supported `6.17` kernel and keep the kernel packages locked.

