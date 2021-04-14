#!/usr/bin/env bash

while true; do raspivid -a 12 -t 0 -w 640 -h 480 -hf -ih -fps 30 -o udp://10.22.23.227:5000 -cd H264 -n; sleep 5; done
