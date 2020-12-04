#!/bin/bash

cd src/
mv methods.cpp methods.cu
mv main.cpp main.cu
cd ../

nvcc -DGPU=1 -I./src -std=c++11 -g -lGL -lGLU -c -o src/main.o src/main.cu
nvcc -DGPU=1 -I./src -std=c++11 -g -lGL -lGLU -c -o src/methods.o src/methods.cu
nvcc -DGPU=1 ./src/main.o ./src/methods.o -lglut -lGL -lGLU -o simulator
