#!/usr/bin/env bash
udp_ip=$1
#while true; do sleep 2; raspivid -a 12 -t 0 -b 1000000 -fps 24 -w 640 -h 480 -o udp://$udp_ip -ih -fl -stm -if adaptive -pf main -n 1> /dev/null; done
while true; do sleep 2; raspivid -a 12 -t 0 -b 8000000 -fps 24 -w 640 -h 480 -o udp://$udp_ip -n; done
