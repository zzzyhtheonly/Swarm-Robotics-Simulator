# How to use
nvcc  -I./src -std=c++11 -g -lGL -lGLU  -c -o src/main.o src/main.cu
<br>
nvcc  -I./src -std=c++11 -g -lGL -lGLU  -c -o src/methods.o src/methods.cpp
<br>
nvcc  ./src/main.o ./src/methods.o -lglut -lGL -lGLU -o simulator