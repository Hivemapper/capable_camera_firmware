#!/bin/bash

trap "i2cset -y 1 20 12 0x00 b" EXIT

while :  
do  
sleep 1
value=$(i2cget -y 1 20 15 b)
if [ "$value" == "0xff" ]  
then
	echo $value
	echo "GPS Locked"
	break
fi
done  

i2cset -y 1 20 12 0xFF b
setarch linux32 ./build/libcamera-bridge --codec mjpeg --segment 0 -o sck:///tmp/bridge.sock --width 4056 --height 2160 --framerate 10 --tuning-file imx477.json --timeout 0
