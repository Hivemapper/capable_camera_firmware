#!/bin/bash

mjpg_streamer -i "input_file.so -d 0.05 -f /tmp/recording/preview/" -o "output_http.so -p 9001"
killall -9 mjpg_streamer
