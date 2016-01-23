#!/bin/bash

cp patches/patch1 OpenTLD
cd OpenTLD
git apply ./patch1
mkdir -p build
cd build
cmake ..
make -j4