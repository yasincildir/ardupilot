#!/bin/bash

echo "Building POSHOLD obstacle avoidance test..."
g++ -std=c++11 -O2 poshold_avoidance_test.cpp -o poshold_avoidance_test -lm

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo ""
    echo "Running POSHOLD avoidance test..."
    echo ""
    ./poshold_avoidance_test
else
    echo "Build failed!"
    exit 1
fi
