
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


