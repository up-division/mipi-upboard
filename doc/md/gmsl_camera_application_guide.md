# <b>GMSL Camera Application Guide</b>

This document explains how image data travels from a GMSL camera module into the Intel SoC, how the Intel IPU6 ISYS capture path is used, how to inspect and configure the media topology, how to operate V4L2 video nodes, and how to build userspace applications with GStreamer, Python, or C/C++.

The current project uses the Intel IPU6 **ISYS capture path only**. Image data is captured from the camera and delivered to userspace through standard V4L2 buffers. The IPU6 PSYS image-processing path is not used.

---

## <b>1. Image Flow from the Camera to the Display</b>

The complete image path is shown below:

The hardware and software stages are:

```text
ISX031 camera sensor
        |
        | MIPI CSI-2
        v
MAX96717F serializer
        |
        | GMSL2 cable
        v
MAX96724 deserializer / MIPI Raw Cam(IMX708、IMX219、OV5647)
        |
        | MIPI CSI-2
        v
Intel IPU6 CSI-2 receiver and D-PHY
        |
        v
Intel IPU6 ISYS capture pipeline
        |
        | DMA
        v
/dev/videoX V4L2 video node
        |
        v
V4L2 userspace buffer
        |
        v
GStreamer / Python / C / C++ application
        |
        v
Demosaic, color conversion, processing, display, or recording
```

### <b>1.1 Camera and Serializer</b>

The ISX031 image sensor produces image data. The MAX96717F serializer receives this data and converts it into a GMSL2 stream for transmission over the GMSL cable.

When streaming is enabled, the current driver writes `0x10` to serializer register `0x02BE` for the serializer corresponding to the selected link.

The serializer address mapping is:

| GMSL Link | Serializer Address |
|---|---:|
| Link 0 | `0x41` |
| Link 1 | `0x42` |
| Link 2 | `0x43` |
| Link 3 | `0x44` |

### <b>1.2 Deserializer and MIPI CSI-2 Output</b>

The MAX96724 receives one or more GMSL2 links, maps them to internal video pipes, and outputs the image data through a MIPI CSI-2 PHY.

The MAX96724 configuration determines:

- Which GMSL links are enabled
- Link-to-pipe mapping
- Virtual-channel mapping
- CSI-2 data type
- MIPI lane count
- MIPI output PHY
- MIPI output frequency
- Deskew configuration

The physical MIPI output routing of the DES board must match the PHY configuration in `max96724.c`.

### <b>1.3 Intel IPU6 ISYS Capture</b>

The Intel IPU6 CSI-2 receiver accepts the MIPI CSI-2 signal from the MAX96724. ISYS receives the CSI-2 packets, identifies frames, transfers the frame data into memory through DMA, and exposes the captured data through V4L2 video nodes such as `/dev/video0` or `/dev/video32`.

### <b>1.4 Userspace Processing and Display</b>

The captured frame may still be raw Bayer data. A userspace application may need to perform:

- Bayer unpacking
- Bit-depth conversion
- Demosaicing
- White-balance correction
- Color correction
- RGB or BGR conversion
- Scaling
- Overlay drawing
- Recording
- AI inference
- Display output

Because this project bypasses PSYS, these operations are performed by GStreamer elements, OpenCV, custom C/C++ code, GPU processing, or another userspace framework.

---

## <b>2. Intel IPU6 ISYS and PSYS</b>

Intel IPU6 contains multiple functional blocks. The two most relevant blocks for camera development are ISYS and PSYS.

### <b>2.1 ISYS</b>

ISYS means **Input System**. It is responsible for receiving and capturing incoming camera data.

The main ISYS functions include:

- Receiving MIPI CSI-2 packets
- Handling the CSI-2 receiver and PHY
- Detecting frame start and frame end
- Receiving virtual channels and CSI-2 data types
- Transferring image data into system memory through DMA
- Managing capture buffers
- Exposing capture devices through V4L2 video nodes

The simplified ISYS path is:

```text
MAX96724 MIPI CSI-2 output
        |
        v
Intel CSI-2 receiver
        |
        v
ISYS DMA capture
        |
        v
V4L2 buffer
        |
        v
Userspace application
```

ISYS mainly captures the image. It does not necessarily convert raw Bayer data into a final display-ready RGB image.

### <b>2.2 PSYS</b>

PSYS means **Processing System**. It is intended for programmable image-processing pipelines after image capture.

Depending on the platform, firmware, and software stack, PSYS may be used for operations such as:

- Demosaicing
- Noise reduction
- Color correction
- Lens-shading correction
- White-balance processing
- Gamma correction
- Scaling
- Format conversion
- Other ISP-related processing

A full PSYS path is conceptually similar to:

```text
Camera
  |
  v
ISYS capture
  |
  v
PSYS image processing
  |
  v
Processed RGB or YUV image
  |
  v
Application
```

### <b>2.3 Why This Project Does Not Use PSYS</b>

This project uses only ISYS because the main objective is to capture image data reliably from the GMSL camera and deliver it to userspace through V4L2.

PSYS is not required for this objective for the following reasons:

1. The camera data can already be captured through ISYS and a V4L2 video node.
2. Userspace applications can process the raw frame with GStreamer, OpenCV, custom code, GPU libraries, or AI frameworks.
3. Bypassing PSYS reduces dependency on PSYS firmware, processing graphs, and platform-specific image-processing components.
4. The capture path is easier to debug because the image is transferred directly from ISYS to a V4L2 buffer.
5. The same raw frame can be processed differently by different applications without changing the kernel pipeline.

The project path is therefore:

```text
Camera -> GMSL -> MAX96724 -> MIPI CSI-2 -> ISYS -> V4L2 Buffer -> Application
```

instead of:

```text
Camera -> GMSL -> MAX96724 -> MIPI CSI-2 -> ISYS -> PSYS -> Application
```

### <b>2.4 Consequences of Bypassing PSYS</b>

Bypassing PSYS provides flexibility, but the application becomes responsible for image processing.

For a raw Bayer stream, the application may need to know:

- Bayer order, such as RGGB, BGGR, GBRG, or GRBG
- Pixel bit depth, such as 8-bit, 10-bit, or 12-bit
- Whether the format is packed or stored in 16-bit words
- Image width and height
- `bytesperline`
- `sizeimage`
- Color-conversion method

Before developing an application, inspect the active V4L2 format with:

```bash
v4l2-ctl -d /dev/videoX --all
```

and:

```bash
v4l2-ctl -d /dev/videoX --get-fmt-video
```

---

## <b>3. Understanding the Media Controller Topology</b>

The Linux Media Controller API represents the camera pipeline as a graph.

The graph contains:

- **Entities:** Hardware blocks or software-visible functional blocks
- **Pads:** Input or output connection points of an entity
- **Links:** Connections between pads
- **Routes:** Internal stream routing inside a multiplexed sub-device
- **Formats:** Media-bus format, resolution, field, and frame interval on a pad

### <b>3.1 Entity</b>

An entity represents one component in the camera pipeline.

Typical entities include:

- `max96724`
- `Intel IPU6 CSI2 0`
- `Intel IPU6 CSI2 4`
- Intel IPU6 capture entities
- `/dev/videoX` capture nodes
- `/dev/v4l-subdevX` sub-device nodes

Example:

```text
- entity 12: max96724 (5 pads, 5 links)
              type V4L2 subdev subtype Sensor flags 0
              device node name /dev/v4l-subdev3
```

### <b>3.2 Pad</b>

A pad is an input or output point of an entity.

- **Sink pad:** Receives data
- **Source pad:** Sends data

Example:

```text
pad0: Sink
pad1: Source
```

### <b>3.3 Link</b>

A link connects a source pad to a sink pad.

Example:

```text
"max96724":4 -> "Intel IPU6 CSI2 4":0 [ENABLED]
```

The arrow indicates the data direction:

```text
source entity:source pad -> sink entity:sink pad
```

Common link flags include:

| Flag | Meaning |
|---|---|
| `[ENABLED]` | The link is active |
| `[IMMUTABLE]` | The link cannot be changed by userspace |
| `[ENABLED,IMMUTABLE]` | The link is active and fixed by the driver |
| No `ENABLED` flag | The link is currently disabled |

### <b>3.4 Format</b>

A pad format describes the image data passing through that pad.

Example:

```text
[fmt:SRGGB10_1X10/1920x1080 field:none]
```

The main fields are:

| Field | Meaning |
|---|---|
| `SRGGB10_1X10` | Raw Bayer media-bus format |
| `1920x1080` | Frame width and height |
| `field:none` | Progressive image |

The source and sink formats on a connected link normally need to be compatible.

### <b>3.5 How to Trace a Video Node</b>

Start from the video node entity and follow each upstream link until reaching the MAX96724 entity.

For example:

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

When streaming from `/dev/video32`, verify that:

1. `/dev/video32` belongs to the expected capture entity.
2. The capture entity is connected to the correct IPU6 CSI-2 receiver.
3. The IPU6 CSI-2 receiver is connected to the expected MAX96724 entity.
4. Required links are marked `[ENABLED]`.
5. The pad formats match the selected camera mode.

Typical platform mappings are:

| Platform | CSI Port | Video Nodes |
|---|---:|---|
| Meteor Lake / Arrow Lake | CSI 0 | `/dev/video0`–`/dev/video3` |
| Meteor Lake / Arrow Lake | CSI 4 | `/dev/video32`–`/dev/video35` |
| Raptor Lake / Alder Lake | CSI 1 | `/dev/video8`–`/dev/video11` |
| Raptor Lake / Alder Lake | CSI 2 | `/dev/video16`–`/dev/video19` |

The exact entity and node numbering may vary between kernel versions, BIOS configurations, and platform revisions.

---

## <b>4. media-ctl Commands</b>

`media-ctl` is used to inspect and configure the Media Controller graph. It does not normally capture image frames. Frame capture is performed through a V4L2 video node.

### <b>4.1 Display Help and Version</b>

```bash
media-ctl --help
media-ctl --version
```

Use the help output installed on the target system as the final reference because available options may differ between `v4l-utils` versions.

### <b>4.2 List Media Devices</b>

```bash
ls -l /dev/media*
```

Inspect every media device:

```bash
for media in /dev/media*; do
    echo "===== $media ====="
    media-ctl -p -d "$media"
done
```

### <b>4.3 Print the Complete Topology</b>

```bash
sudo media-ctl -p -d /dev/media0
```

Equivalent long option:

```bash
sudo media-ctl --print-topology --device /dev/media0
```

### <b>4.4 Print a Specific Entity</b>

```bash
sudo media-ctl -p -d /dev/media0 -e "max96724"
```

Without `-p`, `-e` prints the device node associated with the entity:

```bash
media-ctl -d /dev/media0 -e "max96724"
```

Example output:

```text
/dev/v4l-subdev3
```

### <b>4.5 Filter Relevant Entities</b>

```bash
media-ctl -p -d /dev/media0 | \
awk -v RS= '/- entity [0-9]+: (Intel IPU6 CSI2 0|Intel IPU6 CSI2 1|Intel IPU6 CSI2 2|Intel IPU6 CSI2 4|max967)/'
```

### <b>4.6 Export the Topology as a Graph</b>

Generate a Graphviz DOT file:

```bash
media-ctl --print-dot -d /dev/media0 > topology.dot
```

Convert it into a PNG image:

```bash
dot -Tpng topology.dot -o topology.png
```

Graphviz can be installed with:

```bash
sudo apt install graphviz
```

### <b>4.7 Get a Pad Format</b>

```bash
media-ctl -d /dev/media0 \
  --get-v4l2 '"max96724":4'
```

For a multiplexed stream-aware pad, the syntax may include the stream number:

```bash
media-ctl -d /dev/media0 \
  --get-v4l2 '"max96724":4/0'
```

### <b>4.8 Set a Pad Format</b>

```bash
media-ctl -d /dev/media0 -V \
  '"max96724":4 [fmt:SRGGB10_1X10/1920x1080 field:none]'
```

A frame interval can be included when supported:

```bash
media-ctl -d /dev/media0 -V \
  '"max96724":4 [fmt:SRGGB10_1X10/1920x1080 field:none @1/30]'
```

The driver may modify the requested format. Always print the topology again after setting a format.

### <b>4.9 Enable a Media Link</b>

```bash
media-ctl -d /dev/media0 -l \
  '"max96724":4 -> "Intel IPU6 CSI2 4":0 [1]'
```

`[1]` means enabled.

### <b>4.10 Disable a Media Link</b>

```bash
media-ctl -d /dev/media0 -l \
  '"max96724":4 -> "Intel IPU6 CSI2 4":0 [0]'
```

`[0]` means disabled.

Immutable links cannot be changed from userspace.

### <b>4.11 Reset All Mutable Links</b>

```bash
media-ctl -d /dev/media0 --reset
```

or:

```bash
media-ctl -d /dev/media0 -r
```

This disables all mutable links. Use it carefully because the complete camera pipeline may need to be configured again afterward.

### <b>4.12 Configure Sub-device Routes</b>

Newer Media Controller pipelines may use stream routing inside a sub-device.

Generic syntax:

```bash
media-ctl -d /dev/media0 -R \
  '"entity-name" [sink-pad/sink-stream -> source-pad/source-stream [1]]'
```

Example structure:

```bash
media-ctl -d /dev/media0 -R \
  '"max96724" [0/0 -> 4/0 [1]]'
```

The exact pad and stream numbers must be taken from the actual topology. Routing support depends on the driver and `v4l-utils` version.

### <b>4.13 Select Active or Try State</b>

Use the active configuration:

```bash
media-ctl -d /dev/media0 -W active -p
```

Use the temporary try configuration:

```bash
media-ctl -d /dev/media0 -W try -p
```

`try` settings are useful for checking whether a format is accepted without changing the active hardware configuration.

### <b>4.14 List Known Media-bus Formats</b>

```bash
media-ctl --known-mbus-fmts
```

This command lists symbolic media-bus formats and their numeric values.

### <b>4.15 Interactive Link Configuration</b>

```bash
media-ctl -d /dev/media0 --interactive
```

The tool prompts for links to enable or disable.

### <b>4.16 Verbose Output</b>

```bash
media-ctl -v -p -d /dev/media0
```

Verbose output is useful when a format or link command fails.

### <b>4.17 Recommended media-ctl Verification Sequence</b>

```bash
# 1. Print the complete graph
media-ctl -p -d /dev/media0

# 2. Locate the MAX96724 entity
media-ctl -p -d /dev/media0 -e "max96724"

# 3. Trace the MAX96724 output to the expected IPU6 CSI-2 receiver

# 4. Confirm that every required link is [ENABLED]

# 5. Confirm that the pad formats and resolutions match

# 6. Confirm that the final capture entity maps to the expected /dev/videoX
```

---

## <b>5. V4L2 and v4l2-ctl Commands</b>

V4L2 is the standard Linux userspace API for video capture and control. The `v4l2-ctl` utility exposes most V4L2 operations from the command line.

### <b>5.1 Display All Available v4l2-ctl Options</b>

```bash
v4l2-ctl --help-all
```

The complete option list depends on the installed `v4l-utils` version.

Useful help categories include:

```bash
v4l2-ctl --help
v4l2-ctl --help-vidcap
v4l2-ctl --help-streaming
v4l2-ctl --help-selection
v4l2-ctl --help-subdev
v4l2-ctl --help-io
v4l2-ctl --help-misc
```

### <b>5.2 List All V4L2 Devices</b>

```bash
v4l2-ctl --list-devices
```

Also inspect the device nodes directly:

```bash
ls -l /dev/video*
ls -l /dev/v4l-subdev*
```

### <b>5.3 Select a Video Device</b>

```bash
v4l2-ctl -d /dev/video32 <options>
```

A numeric device index may also be used:

```bash
v4l2-ctl -d 32 <options>
```

### <b>5.4 Show Driver Information</b>

```bash
v4l2-ctl -d /dev/video32 --info
```

This reports the driver name, card name, bus information, capabilities, and device capabilities.

### <b>5.5 Show All Device Information</b>

```bash
v4l2-ctl -d /dev/video32 --all
```

This is one of the most useful commands for debugging because it displays:

- Driver information
- Device capabilities
- Active format
- Resolution
- Pixel format
- `bytesperline`
- `sizeimage`
- Frame rate
- Controls
- Input information

### <b>5.6 List Supported Capture Formats</b>

```bash
v4l2-ctl -d /dev/video32 --list-formats
```

List formats with resolutions and frame intervals:

```bash
v4l2-ctl -d /dev/video32 --list-formats-ext
```

### <b>5.7 List Frame Sizes</b>

```bash
v4l2-ctl -d /dev/video32 \
  --list-framesizes=SRGGB10
```

The accepted FourCC depends on the driver. Use `--list-formats-ext` first.

### <b>5.8 List Frame Intervals</b>

```bash
v4l2-ctl -d /dev/video32 \
  --list-frameintervals=width=1920,height=1080,pixelformat=RG10
```

### <b>5.9 Get the Current Video Format</b>

```bash
v4l2-ctl -d /dev/video32 --get-fmt-video
```

### <b>5.10 Set the Video Format</b>

```bash
v4l2-ctl -d /dev/video32 \
  --set-fmt-video=width=1920,height=1080,pixelformat=RG10
```

Always read the format back because the driver may adjust the requested values:

```bash
v4l2-ctl -d /dev/video32 --get-fmt-video
```

### <b>5.11 Try a Format Without Applying It</b>

```bash
v4l2-ctl -d /dev/video32 \
  --try-fmt-video=width=1920,height=1080,pixelformat=RG10
```

### <b>5.12 Get and Set the Frame Rate</b>

Get the current stream parameters:

```bash
v4l2-ctl -d /dev/video32 --get-parm
```

Request 30 frames per second:

```bash
v4l2-ctl -d /dev/video32 --set-parm=30
```

The actual frame rate is controlled by the complete sensor, GMSL, CSI-2, and IPU configuration. The driver may reject or adjust the requested value.

### <b>5.13 List V4L2 Controls</b>

```bash
v4l2-ctl -d /dev/video32 --list-ctrls
```

Include menu choices:

```bash
v4l2-ctl -d /dev/video32 --list-ctrls-menus
```

Controls may belong to the video node or a sensor sub-device. Not every capture node exposes camera controls directly.

### <b>5.14 Read a Control</b>

```bash
v4l2-ctl -d /dev/video32 --get-ctrl=exposure
```

Read multiple controls:

```bash
v4l2-ctl -d /dev/video32 \
  --get-ctrl=exposure,gain
```

### <b>5.15 Set a Control</b>

```bash
v4l2-ctl -d /dev/video32 --set-ctrl=exposure=800
```

Set multiple controls:

```bash
v4l2-ctl -d /dev/video32 \
  --set-ctrl=exposure=800,gain=16
```

### <b>5.16 Get Crop and Selection Information</b>

List selection-related options available in the installed version:

```bash
v4l2-ctl --help-selection
```

Typical commands include:

```bash
v4l2-ctl -d /dev/video32 \
  --get-selection=target=crop
```

and:

```bash
v4l2-ctl -d /dev/video32 \
  --set-selection=target=crop,left=0,top=0,width=1920,height=1080
```

Selection support depends on the driver.

### <b>5.17 Capture One Frame with MMAP</b>

```bash
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-count=1 \
  --stream-to=frame.raw
```

### <b>5.18 Capture Multiple Frames</b>

```bash
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-count=100 \
  --stream-to=capture.raw
```

Raw frames are written consecutively. Use `sizeimage` from `--all` or `--get-fmt-video` to split the file into individual frames.

### <b>5.19 Stream Continuously to /dev/null</b>

This is useful for testing stability and measuring the actual capture rate without displaying or saving the image:

```bash
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-count=0 \
  --stream-to=/dev/null
```

### <b>5.20 Skip Initial Frames</b>

```bash
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-skip=10 \
  --stream-count=1 \
  --stream-to=frame.raw
```

This discards the first ten frames before saving the next frame.

### <b>5.21 Use poll During Streaming</b>

```bash
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-poll \
  --stream-count=100 \
  --stream-to=/dev/null
```

### <b>5.22 Stream Through Standard Output</b>

```bash
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-count=0 \
  --stream-to=-
```

This writes the captured frame bytes to standard output. Another application can read the data from standard input.

Example:

```bash
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-count=0 \
  --stream-to=- | python3 display_bayer10.py 1920 1080
```

### <b>5.23 V4L2 Streaming I/O Methods</b>

V4L2 supports several buffer methods.

| Method | Description |
|---|---|
| `read()` | Simple read-based capture; not supported by every driver |
| MMAP | Kernel allocates buffers and maps them into userspace |
| USERPTR | Application allocates memory and gives pointers to the driver |
| DMABUF | Buffers are shared through DMA buffer file descriptors |

Common `v4l2-ctl` streaming options include:

```bash
--stream-mmap=<buffer_count>
--stream-user=<buffer_count>
--stream-dmabuf=<buffer_count>
```

Support depends on the device capabilities and driver implementation.

For most initial tests, use:

```bash
--stream-mmap=4
```

For high-performance zero-copy pipelines, DMABUF is preferred when all participating components support it.

### <b>5.24 Check Whether a Video Node Is Busy</b>

```bash
sudo fuser -v /dev/video*
```

Terminate a remaining process:

```bash
sudo kill <PID>
```

Force termination if necessary:

```bash
sudo kill -9 <PID>
```

### <b>5.25 Run V4L2 Compliance Tests</b>

```bash
v4l2-compliance -d /dev/video32
```

A driver does not need to pass every optional test to capture an image, but failures can reveal API implementation problems.

### <b>5.26 Use yavta for Low-level Capture Testing</b>

`yavta` is another useful V4L2 test application.

Example:

```bash
yavta -c100 -n4 -f RG10 -s 1920x1080 /dev/video32
```

The accepted format name depends on the installed `yavta` version and the video node.

---

## <b>6. Understanding V4L2 Buffers</b>

A V4L2 video node delivers captured images through buffers.

A typical MMAP capture sequence is:

```text
open(/dev/videoX)
        |
        v
VIDIOC_QUERYCAP
        |
        v
VIDIOC_S_FMT
        |
        v
VIDIOC_REQBUFS
        |
        v
VIDIOC_QUERYBUF
        |
        v
mmap()
        |
        v
VIDIOC_QBUF
        |
        v
VIDIOC_STREAMON
        |
        v
poll() or select()
        |
        v
VIDIOC_DQBUF
        |
        v
Process the frame
        |
        v
VIDIOC_QBUF
        |
        v
Repeat
        |
        v
VIDIOC_STREAMOFF
```

### <b>6.1 Important Buffer Fields</b>

| Field | Meaning |
|---|---|
| `index` | Buffer index |
| `bytesused` | Number of valid bytes in the captured frame |
| `length` | Allocated buffer length |
| `timestamp` | Capture timestamp |
| `sequence` | Frame sequence number |
| `memory` | MMAP, USERPTR, or DMABUF memory type |

### <b>6.2 bytesperline and sizeimage</b>

Do not assume that one frame always occupies exactly `width × height × bytes_per_pixel`.

The driver may add line padding.

Use:

```bash
v4l2-ctl -d /dev/video32 --get-fmt-video
```

and check:

- `Bytes per Line`
- `Size Image`

Applications should use the reported `bytesperline`, `sizeimage`, and `bytesused` values.

### <b>6.3 Raw 10-bit Formats</b>

A raw 10-bit Bayer format may be stored in different ways:

- Unpacked into 16-bit little-endian words
- Packed as multiple 10-bit pixels across byte boundaries
- Padded to a platform-specific stride

The application must confirm the format layout before converting the image.

---

## <b>7. Application Development Concept</b>

Once an application can receive frame bytes from a V4L2 buffer or standard input, the application is not limited to one language or framework.

The general application model is:

```text
Acquire frame buffer
        |
        v
Interpret pixel format
        |
        v
Convert raw data into an image matrix
        |
        v
Optional processing
        |
        v
Display, save, transmit, or analyze the image
```

Possible application environments include:

- GStreamer
- Python with OpenCV or NumPy
- C with the V4L2 API
- C++ with V4L2 and OpenCV
- Rust V4L2 libraries
- FFmpeg
- CUDA or OpenCL processing
- AI inference frameworks
- Custom network-streaming applications

The essential requirement is the ability to obtain and interpret the V4L2 frame buffer correctly.

---

## <b>8. GStreamer Application</b>

GStreamer builds a multimedia pipeline by connecting elements with `!`.

The basic structure is:

```text
source ! caps ! conversion ! processing ! sink
```

For a V4L2 camera:

```text
v4l2src ! format description ! image conversion ! display sink
```

### <b>8.1 Inspect the v4l2src Element</b>

```bash
gst-inspect-1.0 v4l2src
```

This shows supported properties, including the available I/O modes.

### <b>8.2 Inspect Bayer Conversion Support</b>

```bash
gst-inspect-1.0 bayer2rgb
```

If the raw Bayer format is not accepted by `bayer2rgb`, a custom unpacking or conversion element may be required.

### <b>8.3 Basic Pipeline for a Display-ready Format</b>

If the video node outputs a format already supported by `videoconvert`, use:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video32 ! \
  videoconvert ! \
  autovideosink sync=false
```

### <b>8.4 Set Resolution and Frame Rate</b>

For a YUY2 example:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video32 ! \
  'video/x-raw,format=YUY2,width=1920,height=1080,framerate=30/1' ! \
  videoconvert ! \
  autovideosink sync=false
```

The caps must match the format reported by:

```bash
v4l2-ctl -d /dev/video32 --list-formats-ext
```

### <b>8.5 Raw Bayer Display Pipeline</b>

When the source and installed plugins support the Bayer format:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video32 io-mode=mmap ! \
  'video/x-bayer,format=rggb,width=1920,height=1080,framerate=30/1' ! \
  bayer2rgb ! \
  videoconvert ! \
  autovideosink sync=false
```

Replace `rggb` with the actual Bayer order.

For 10-bit formats, the caps name and plugin support depend on the GStreamer version. If GStreamer cannot negotiate the raw 10-bit Bayer format, capture the data with `appsink` and perform unpacking and demosaicing in the application.

### <b>8.6 Save Raw Frames</b>

Save one buffer:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video32 num-buffers=1 ! \
  filesink location=frame.raw
```

Save a continuous raw stream:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video32 ! \
  filesink location=capture.raw
```

The output file contains raw frame bytes and does not include a standard video container unless an encoder and muxer are added.

### <b>8.7 Use appsink</b>

`appsink` passes each `GstBuffer` to application code.

Example pipeline:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video32 io-mode=mmap ! \
  queue max-size-buffers=2 leaky=downstream ! \
  appsink name=sink sync=false max-buffers=2 drop=true
```

In a real GStreamer application, the program maps the `GstBuffer`, reads the image bytes, processes the frame, and then releases the sample.

### <b>8.8 GStreamer Pipeline Interpretation</b>

Example:

```bash
gst-launch-1.0 -v \
  v4l2src device=/dev/video32 ! \
  'video/x-bayer,format=rggb,width=1920,height=1080,framerate=30/1' ! \
  bayer2rgb ! \
  videoconvert ! \
  autovideosink sync=false
```

| Part | Function |
|---|---|
| `gst-launch-1.0` | Runs a pipeline from the command line |
| `-v` | Prints negotiated capabilities |
| `v4l2src` | Captures from a V4L2 video node |
| `device=/dev/video32` | Selects the camera node |
| `video/x-bayer` | Describes the raw Bayer stream |
| `format=rggb` | Defines the Bayer order |
| `width`, `height` | Define the resolution |
| `framerate=30/1` | Requests 30 FPS |
| `bayer2rgb` | Demosaics Bayer data |
| `videoconvert` | Converts into a sink-compatible format |
| `autovideosink` | Selects a display sink automatically |
| `sync=false` | Displays frames without synchronizing to the pipeline clock |

### <b>8.9 Useful GStreamer Debugging</b>

Display pipeline negotiation:

```bash
GST_DEBUG=2 gst-launch-1.0 -v <pipeline>
```

Increase the debug level:

```bash
GST_DEBUG=4 gst-launch-1.0 -v <pipeline>
```

Check available video sinks:

```bash
gst-inspect-1.0 | grep sink | grep video
```

If `autovideosink` does not work, test another sink such as:

```bash
... ! glimagesink
```

or:

```bash
... ! waylandsink
```

or:

```bash
... ! ximagesink
```

---

## <b>9. Python Application Using Standard Input</b>

A simple Python application can receive raw frame data from the standard output of `v4l2-ctl`.

The data flow is:

```text
V4L2 video node
        |
        v
v4l2-ctl --stream-to=-
        |
        | stdout pipe
        v
Python sys.stdin.buffer
        |
        v
NumPy image array
        |
        v
OpenCV demosaic and display
```

### <b>9.1 Start the Pipeline</b>

```bash
v4l2-ctl -d /dev/video32 \
  --set-fmt-video=width=1920,height=1080,pixelformat=RG10 \
  --stream-mmap=4 \
  --stream-count=0 \
  --stream-to=- | \
python3 display_bayer10.py 1920 1080
```

### <b>9.2 Python Example</b>

The following example assumes that each 10-bit Bayer pixel is stored in one 16-bit little-endian word.

```python
#!/usr/bin/env python3

import sys
from typing import BinaryIO

import cv2
import numpy as np


def read_exact(stream: BinaryIO, size: int) -> bytes:
    """Read exactly size bytes, or return b'' when the pipe is closed."""
    data = bytearray()

    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            return b""
        data.extend(chunk)

    return bytes(data)


def main() -> int:
    if len(sys.argv) != 3:
        print(
            f"Usage: {sys.argv[0]} <width> <height>",
            file=sys.stderr,
        )
        return 1

    width = int(sys.argv[1])
    height = int(sys.argv[2])

    # This example assumes 16 bits are used to store each 10-bit pixel.
    frame_bytes = width * height * 2

    while True:
        frame = read_exact(sys.stdin.buffer, frame_bytes)
        if not frame:
            break

        raw16 = np.frombuffer(frame, dtype="<u2").reshape(height, width)

        # Convert the 10-bit range 0-1023 into 8-bit data.
        raw8 = np.clip(raw16, 0, 1023).astype(np.float32)
        raw8 = (raw8 * (255.0 / 1023.0)).astype(np.uint8)

        # Change the conversion code if the Bayer order is not RGGB.
        bgr = cv2.cvtColor(raw8, cv2.COLOR_BayerRG2BGR)

        cv2.imshow("GMSL Camera", bgr)
        if cv2.waitKey(1) & 0xFF in (27, ord("q")):
            break

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

Install the required Python packages:

```bash
sudo apt install python3-opencv python3-numpy
```

### <b>9.3 Important Python Notes</b>

This example must be modified if:

- The Bayer order is not RGGB
- The format is packed RAW10
- `bytesperline` contains padding
- `sizeimage` is larger than `width × height × 2`
- The video node uses a multi-planar format

The standard-output method is easy to understand, but it introduces extra copies. For higher performance, use direct V4L2 MMAP, GStreamer `appsink`, or DMABUF.

---

## <b>10. C/C++ Application Using V4L2 MMAP</b>

C and C++ applications can access V4L2 directly through `ioctl()` calls.

The following example:

1. Opens a video node.
2. Sets an RGGB10 format.
3. Requests MMAP buffers.
4. Starts streaming.
5. Dequeues each captured buffer.
6. Converts the 10-bit Bayer image into BGR with OpenCV.
7. Displays the image.
8. Requeues the buffer.

### <b>10.1 C++ Example</b>

```cpp
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

struct MappedBuffer {
    void* start = MAP_FAILED;
    std::size_t length = 0;
};

static int xioctl(int fd, unsigned long request, void* arg) {
    int result;

    do {
        result = ioctl(fd, request, arg);
    } while (result == -1 && errno == EINTR);

    return result;
}

static void throw_errno(const std::string& message) {
    throw std::runtime_error(message + ": " + std::strerror(errno));
}

int main(int argc, char** argv) {
    const std::string device = argc > 1 ? argv[1] : "/dev/video32";
    const uint32_t width = argc > 2 ? std::stoul(argv[2]) : 1920;
    const uint32_t height = argc > 3 ? std::stoul(argv[3]) : 1080;

    int fd = open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        throw_errno("Failed to open " + device);
    }

    try {
        v4l2_capability cap{};
        if (xioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            throw_errno("VIDIOC_QUERYCAP failed");
        }

        if (!(cap.device_caps & V4L2_CAP_VIDEO_CAPTURE)) {
            throw std::runtime_error("The device is not a single-planar capture node");
        }

        if (!(cap.device_caps & V4L2_CAP_STREAMING)) {
            throw std::runtime_error("The device does not support streaming I/O");
        }

        v4l2_format format{};
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB10;
        format.fmt.pix.field = V4L2_FIELD_NONE;

        if (xioctl(fd, VIDIOC_S_FMT, &format) < 0) {
            throw_errno("VIDIOC_S_FMT failed");
        }

        std::cout << "Active format: "
                  << format.fmt.pix.width << "x" << format.fmt.pix.height
                  << ", bytesperline=" << format.fmt.pix.bytesperline
                  << ", sizeimage=" << format.fmt.pix.sizeimage
                  << std::endl;

        v4l2_requestbuffers request{};
        request.count = 4;
        request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        request.memory = V4L2_MEMORY_MMAP;

        if (xioctl(fd, VIDIOC_REQBUFS, &request) < 0) {
            throw_errno("VIDIOC_REQBUFS failed");
        }

        if (request.count < 2) {
            throw std::runtime_error("Insufficient V4L2 buffers");
        }

        std::vector<MappedBuffer> buffers(request.count);

        for (uint32_t index = 0; index < request.count; ++index) {
            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;
            buffer.index = index;

            if (xioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
                throw_errno("VIDIOC_QUERYBUF failed");
            }

            buffers[index].length = buffer.length;
            buffers[index].start = mmap(
                nullptr,
                buffer.length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd,
                buffer.m.offset
            );

            if (buffers[index].start == MAP_FAILED) {
                throw_errno("mmap failed");
            }

            if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
                throw_errno("VIDIOC_QBUF failed");
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (xioctl(fd, VIDIOC_STREAMON, &type) < 0) {
            throw_errno("VIDIOC_STREAMON failed");
        }

        bool running = true;

        while (running) {
            pollfd pfd{};
            pfd.fd = fd;
            pfd.events = POLLIN;

            const int poll_result = poll(&pfd, 1, 2000);
            if (poll_result == 0) {
                std::cerr << "Timed out while waiting for a frame" << std::endl;
                continue;
            }

            if (poll_result < 0) {
                if (errno == EINTR) {
                    continue;
                }
                throw_errno("poll failed");
            }

            v4l2_buffer buffer{};
            buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buffer.memory = V4L2_MEMORY_MMAP;

            if (xioctl(fd, VIDIOC_DQBUF, &buffer) < 0) {
                if (errno == EAGAIN) {
                    continue;
                }
                throw_errno("VIDIOC_DQBUF failed");
            }

            if (buffer.index >= buffers.size()) {
                throw std::runtime_error("Invalid V4L2 buffer index");
            }

            const int active_width = static_cast<int>(format.fmt.pix.width);
            const int active_height = static_cast<int>(format.fmt.pix.height);
            const std::size_t stride = format.fmt.pix.bytesperline;

            // The example assumes unpacked RAW10 stored in 16-bit words.
            cv::Mat raw16(
                active_height,
                active_width,
                CV_16UC1,
                buffers[buffer.index].start,
                stride
            );

            cv::Mat raw8;
            raw16.convertTo(raw8, CV_8U, 255.0 / 1023.0);

            cv::Mat bgr;
            cv::cvtColor(raw8, bgr, cv::COLOR_BayerRG2BGR);

            cv::imshow("GMSL Camera", bgr);
            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                running = false;
            }

            if (xioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
                throw_errno("VIDIOC_QBUF failed");
            }
        }

        if (xioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
            throw_errno("VIDIOC_STREAMOFF failed");
        }

        for (const auto& buffer : buffers) {
            if (buffer.start != MAP_FAILED) {
                munmap(buffer.start, buffer.length);
            }
        }

        close(fd);
        return 0;
    } catch (...) {
        close(fd);
        throw;
    }
}
```

### <b>10.2 Compile the C++ Example</b>

```bash
g++ -std=c++17 -O2 -Wall -Wextra \
  v4l2_display.cpp \
  -o v4l2_display \
  $(pkg-config --cflags --libs opencv4)
```

Run it with:

```bash
./v4l2_display /dev/video32 1920 1080
```

### <b>10.3 C/C++ Implementation Notes</b>

The example assumes:

- Single-planar `V4L2_BUF_TYPE_VIDEO_CAPTURE`
- `V4L2_PIX_FMT_SRGGB10`
- RAW10 stored in 16-bit words
- RGGB Bayer order

The implementation must be changed if the node uses:

- `V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE`
- Packed RAW10
- Another Bayer order
- Another bit depth
- Multiple planes
- DMABUF instead of MMAP

For a production application, also handle:

- Timeouts
- Device disconnection
- Stream restart
- Frame sequence loss
- Buffer timestamps
- Graceful signal handling
- Correct shutdown with `VIDIOC_STREAMOFF`

---

## <b>11. Comparison of Application Methods</b>

| Method | Advantages | Limitations |
|---|---|---|
| GStreamer command | Fastest way to test and display a pipeline | Raw 10-bit Bayer plugin support may vary |
| GStreamer application with appsink | Flexible pipeline and application access to `GstBuffer` | Requires GStreamer API knowledge |
| Python through stdin | Simple proof-of-concept and easy image processing | Additional copies and lower performance |
| Python OpenCV V4L2 backend | Very simple application code | Format negotiation and raw Bayer support may be limited |
| C/C++ V4L2 MMAP | Full control and good performance | More code and manual buffer management |
| DMABUF pipeline | Enables low-copy or zero-copy processing | Requires support from every component |

---

## <b>12. Recommended Application Development Sequence</b>

Use the following sequence when developing a new application:

```text
1. Verify the camera with i2cdetect
        |
        v
2. Verify the media topology with media-ctl -p
        |
        v
3. Verify the node format with v4l2-ctl --all
        |
        v
4. Capture frames to /dev/null
        |
        v
5. Save one raw frame
        |
        v
6. Verify Bayer order, bit depth, stride, and sizeimage
        |
        v
7. Display the frame with GStreamer, Python, or C/C++
        |
        v
8. Add processing, recording, networking, or AI functions
```

Recommended commands:

```bash
# Check the topology
media-ctl -p -d /dev/media0

# Inspect the video node
v4l2-ctl -d /dev/video32 --all

# Test continuous capture
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-count=300 \
  --stream-to=/dev/null

# Save one frame
v4l2-ctl -d /dev/video32 \
  --stream-mmap=4 \
  --stream-count=1 \
  --stream-to=frame.raw
```

---

## <b>13. Important Notes</b>

1. `media-ctl` configures and inspects the media graph; `v4l2-ctl` controls and captures from a video node.
2. A valid `/dev/videoX` node does not guarantee that the complete upstream media path is enabled.
3. A valid media topology does not guarantee that the camera is producing frames.
4. Because PSYS is bypassed, raw Bayer conversion is normally performed in userspace.
5. Always verify the actual Bayer order, bit depth, `bytesperline`, and `sizeimage` before interpreting a buffer.
6. Use MMAP for initial development and DMABUF when a low-copy pipeline is required.
7. Stop the stream correctly before closing an application.
8. If a program crashes or remains active, check the video nodes with `sudo fuser -v /dev/video*`.
9. Different kernel, GStreamer, and `v4l-utils` versions may expose different formats or command options.
10. Use `media-ctl --help` and `v4l2-ctl --help-all` on the target system as the final command reference.
