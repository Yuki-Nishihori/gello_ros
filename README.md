
# Get offset for Gello robot
## for UR
python3 scripts/gello_get_offset.py --start-joints -3.14 -1.57 -1.57 -1.57 1.57 0 --joint-signs 1 1 -1 1 1 1  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0
python3 scripts/gello_get_offset.py --start-joints -1.57 -1.57 -1.57 -1.57 1.57 0 --joint-signs 1 1 -1 1 1 1  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0
## for cobotta
python3 scripts/gello_get_offset.py --start-joints 0 0 1.57 0 1.57 0 --joint-signs 1 1 -1 1 -1 1  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YXAT-if00-port0
## for FR3
python3 scripts/gello_get_offset.py --start-joints -1.57 -1.57 -1.57 -1.57 1.57 0 --joint-signs 1 1 -1 1 1 1  --port /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISUQE-if00-port0

## Serial
/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT88YXAT-if00-port0 



## Check serial port
```
ls /dev/serial/by-id/
```
## Output
```
best offsets               :  ['9.425', '4.712', '3.142', '4.712', '1.571', '1.571']
best offsets function of pi: [6*np.pi/2, 3*np.pi/2, 2*np.pi/2, 3*np.pi/2, 1*np.pi/2, 1*np.pi/2 ]
gripper open (degrees)        113.091015625
gripper close (degrees)       71.291015625
```

## boudrate
4M -> too fast, unstable communication
2M -> stable communication

## Imitation learning by ACT
The Action Chunking with Transformer (ACT) scripts are referenced from the [Github repository.](https://github.com/Shaka-Labs/ACT)
These scripts have been simplified for single-handed use, in contrast to the bimanual focus of the original ACT repository.


## 対応カメラ
1. Realsense
   - "realsense_ros"を使う
2. UVC
   - "libuvc_camera"を使う
### USBカメラの指定可能なフォーマットの調べ方
```
root@onolab_ros:~/onolab/catkin_ws# v4l2-ctl --device /dev/video0 --list-formats-ext
ioctl: VIDIOC_ENUM_FMT
	Type: Video Capture

	[0]: 'MJPG' (Motion-JPEG, compressed)
		Size: Discrete 2048x1536
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 1920x1080
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 1024x768
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 640x480
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 1280x960
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 1280x720
			Interval: Discrete 0.033s (30.000 fps)
		Size: Discrete 800x600
			Interval: Discrete 0.033s (30.000 fps)
```
### USBカメラのベンダーIDとプロダクトIDの調べ方

This guide explains how to find the **vendor ID** and **product ID** of a USB camera using Linux commands. These IDs are necessary when configuring camera drivers such as `libuvc_camera` in ROS.

---

#### 1. Check Device Information Using `udevadm`
First, use the `udevadm` command to get detailed information about the camera device.

```bash
udevadm info --name=/dev/video0
```
#### Example Output
```bash
E: ID_VENDOR_ID=04f2
E: ID_MODEL_ID=1601
E: ID_VENDOR=USB_Camera_L-837
E: ID_MODEL=USB_Camera_L-837
```
From this output:
- **Vendor ID** → `04f2`
- **Product ID** → `1601`

---

#### 2. List USB Devices Using `lsusb`
The `lsusb` command shows all connected USB devices along with their IDs.

```bash
lsusb
```
#### Example Output
```bash
Bus 007 Device 004: ID 04f2:1601 Chicony Electronics Co., Ltd
```
The format is:
```
ID <Vendor ID>:<Product ID>
```
In this case:
- **Vendor ID** → `04f2`
- **Product ID** → `1601`

---

#### 3. How to Use These IDs in ROS
When configuring ROS camera drivers like `libuvc_camera`, set the IDs in your `.launch` file as follows:
```xml
<param name="vendor" value="0x04f2" />
<param name="product" value="0x1601" />
```
---

