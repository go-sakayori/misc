#!/bin/bash

# Ensure the script is run with root privileges
if [ "$EUID" -ne 0 ]; then 
  echo "Please run this script with sudo."
  exit 1
fi

echo "Applying network optimizations..."

# Increase the maximum receive buffer size
sysctl -w net.core.rmem_max=2147483647

# IP fragmentation settings
sysctl -w net.ipv4.ipfrag_time=3
sysctl -w net.ipv4.ipfrag_high_thresh=134217728

echo "Optimization complete."
