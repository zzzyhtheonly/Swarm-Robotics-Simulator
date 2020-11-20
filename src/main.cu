#include <GL/gl.h>
#include <iostream>
#include <fstream>
#include <cuda.h>

#include "headers.h"

/* arguments */
unsigned int population_size = 10;
double radius = 20;
double ground_dimension = 1000;

/* those are fixed at the moment */
unsigned int dimension_size = 2;
__global__ void gpu_move_kernel(vector<vector<double>> *, vector<vector<double>> *, double, unsigned int, unsigned int);

void clear_screen()
{
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
}

void render_function()
{
        clear_screen();
	glColor3f(0.0, 0.0, 1.0);
        glOrtho(-ground_dimension, ground_dimension, -ground_dimension, ground_dimension, -ground_dimension, ground_dimension);

        /* draw something */
        population test = population(population_size, dimension_size, radius, ground_dimension);
        dim3 blocksPerGrid(ceil(test.pop_size/16.0), ceil(test.pop_size/16.0), 1);
        dim3 threadsPerBlock(16, 16, 1);
        vector<vector<double>> * posD, * velocityD;
        vector<vector<double>> pos, velocity;
        for(unsigned int i = 0; i < test.pop_size; i++){
                pos.push_back(test.entities[i].pos);
                velocity.push_back(test.entities[i].velocity);
                test.entities[i].move();
         }
        for (unsigned int timestamp = 0; timestamp < 200; timestamp++){
                // gpu_move_kernel<<blocksPerGrid, threadsPerBlock>>();
                for(unsigned int i = 0; i < test.pop_size; ++i){
                        test.entities[i].draw();
                        test.entities[i].move_prediction();
                }

                test.adjustment();

                /* TODO: GPU version */
cudaMemcpy(posD, &pos, pos.size() * pos[0].size() * sizeof(double), cudaMemcpyHostToDevice);
                cudaMemcpy(velocityD, &velocity, velocity.size() * velocity[0].size() * sizeof(double), cudaMemcpyHostToDevice);
                gpu_move_kernel<<<blocksPerGrid, threadsPerBlock>>>(posD, velocityD, test.entities[1].limit, test.entities[0].dimension, population_size);
                cudaMemcpy(&pos, posD, pos.size() * pos[0].size() * sizeof(double), cudaMemcpyDeviceToHost);
                cudaMemcpy(&velocity, velocityD, velocity.size() * velocity[0].size() * sizeof(double), cudaMemcpyDeviceToHost);
                for(unsigned int i = 0; i < test.pop_size; i++){
                        test.entities[i].velocity = velocity[i];
                        test.entities[i].pos = pos[i];
                        test.entities[i].pos_next = pos[i];
                }
                glFlush();
                clear_screen();
        }

        ofstream out("out.txt");
        if (out.is_open()) {
            for(unsigned int i = 0; i < test.pop_size; ++i){
                                out << "(" << test.entities[i].pos[0] << ","
                                << test.entities[i].pos[1] << ")";
                }
        out.close();
    }
        glFlush();
}

int main(int argc, char* argv[])
{
        /* parse arguments */
        if(argc == 1){
                std::cout << "Usage: ./simulator [population size] "
                          << "[radius of individual: default is set to 20] "
                          << "[dimension of the playground: default is set to 1000]"
                          << std::endl;
                std::cout << "Example: ./simulator 10" << std::endl;
                std::cout << "         ./simulator 50 50 2000 " << std::endl;
                std::cout << "Note: exit the program by entering Ctrl^C from the terminal" << std::endl;
                return 0;
        }
        if(argc >= 2){
                population_size = atoi(argv[1]);
        }
        if(argc >= 3){
                radius = atof(argv[2]);
        }
        if(argc >= 4){
                ground_dimension = atof(argv[3]);
        }

        render_function();

        return 0;
}

__global__ void gpu_move_kernel(vector<vector<double>> * posD, vector<vector<double>> * velocityD, double limit, unsigned int dimension, unsigned int size){
        unsigned int col = blockIdx.x * blockDim.x + threadIdx.x;
        unsigned int row = blockIdx.y * blockDim.y + threadIdx.y;
        for(unsigned int i = 0; i < dimension; i++){
                /* detect collision with walls */
                double tmp = posD[row*size+col][i] + velocityD[row*size+col][i];
                if(tmp > limit || tmp < -limit){
                        velocityD[row*size+col][i] = -velocityD[row*size+col][i];
                }
                posD[row*size+col][i] += velocityD[row*size+col][i];
        }
        return;
}

