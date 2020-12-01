# How to use
nvcc  -I./src -std=c++11 -g -lGL -lGLU  -c -o src/main.o src/main.cu
<br>
nvcc  -I./src -std=c++11 -g -lGL -lGLU  -c -o src/methods.o src/methods.cpp
<br>
nvcc  ./src/main.o ./src/methods.o -lglut -lGL -lGLU -o simulator

nvcc -std=c++11 -g -lGL -lGLU  -c -o main.o main.cu
<br>
nvcc -std=c++11 -g -lGL -lGLU  -c -o methods.o methods.cpp
<br>
nvcc  main.o methods.o -lglut -lGL -lGLU -o simulator
