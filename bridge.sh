#!/bin/bash

trap "i2cset -y 1 0x14 0xc 0x00 b" EXIT

while :  
do  
sleep 1
value=$(i2cget -y 1 0x14 0xf b)
if [ "$value" == "0xff" ]  
then
    echo $value
    echo "GPS Locked"
    break
fi
done  

i2cset -y 1 0x14 0xc 0xFF b
setarch linux32 ./libcamera-bridge --tuning-file imx477.json --segment 0 --netconfig
