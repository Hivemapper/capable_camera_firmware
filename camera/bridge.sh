#!/bin/bash

if [ ! -d "/mnt/data/pic" ]
then
  mkdir /mnt/data/pic
fi

./libcamera-bridge --config config.json --segment 0  --timeout 0 --tuning-file imx477.json --quality 70
