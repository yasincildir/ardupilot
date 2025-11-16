#!/bin/bash

echo "Building comprehensive test suite..."
g++ -std=c++11 -O2 comprehensive_indoor_test.cpp -o comprehensive_indoor_test -lm

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo ""
    echo "Running comprehensive tests..."
    echo ""
    ./comprehensive_indoor_test
else
    echo "Build failed!"
    exit 1
fi
