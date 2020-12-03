#!/bin/bash

cd src/
rm -f *.o
mv main.cu main.cpp
mv methods.cu methods.cpp
cd ..
rm -f simulator out.tga
