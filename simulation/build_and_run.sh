#!/bin/bash

# Build and run ArduPilot Indoor Flight Simulation

echo "Building simulation..."
g++ -std=c++11 -O2 -Wall -o indoor_flight_test indoor_flight_test.cpp -lm

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo ""
    echo "Running simulation..."
    echo ""
    ./indoor_flight_test
else
    echo "Build failed!"
    exit 1
fi
