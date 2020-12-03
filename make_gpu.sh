#!/bin/bash

nvcc -DGPU=1 -I./src -std=c++11 -g -lGL -lGLU -c -o src/main.o src/main.cpp
nvcc -DGPU=1 -I./src -std=c++11 -g -lGL -lGLU -c -o src/methods.o src/methods.cpp
nvcc -DGPU=1 ./src/main.o ./src/methods.o -lglut -lGL -lGLU -o simulator