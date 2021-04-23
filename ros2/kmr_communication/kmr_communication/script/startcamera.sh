#!/usr/bin/env bash
udp_ip=$1
while true; do raspivid -a 12 -t 0 -b 3000000 -fps 24 -w 640 -h 480 -o udp://$udp_ip -ih -fl -stm -if adaptive -pf main -n; sleep 1; done