#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <cuda.h>

#include <thrust/device_vector.h>

/* states describe a robot's status */
enum states
{
  TERMINATE, // 0
  STOP, // 1
  READY, // 2
  RUNNING, // 3
  ON_OBJ, 
  LINK, 
  SENSE, // 6
  PATH,
  BRANCH, //8
};

__device__ __host__ void print_pos(double*, double*, int);


__device__ __host__ void move(unsigned int, double *, double *, double * , double *, int *, double);
__device__ __host__ bool _move(unsigned int , double *, double *, double *, double *, double );

// This is the move that is launched from CPU and GPU runs it for each cell
__global__ void move_kernel(double *position_x, double *position_y, double *velocity_x, double *velocity_y, int *status, int pop_size)
{
    unsigned int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index >= pop_size) return;
    move(index, position_x, position_y, velocity_x, velocity_y, status, 1000.0);
    // position_x[index] += velocity_x[index];
    // position_y[index] += velocity_y[index];
    printf("%d, %f %f\n", index, position_x[index], position_y[index]);
    // positions[index] = index; // use this one for debugging the index
}

__global__ void kernel1(double *positions, double *velocities)
{
    unsigned int index = blockDim.x * blockIdx.x + threadIdx.x;
    positions[index] += velocities[index];
    // positions[index] = index; // use this one for debugging the index
}


class RobotSystem
{
  public:
    RobotSystem() = delete;
    RobotSystem(unsigned int numRobots){
      pop_size = numRobots;
    };
    void birth_robot();       // pushes back one more robot data to the device_vectors
    void advance_robot();    // launches the move that adds velocity to positions
    thrust::device_vector<double> position_x;
    thrust::device_vector<double> position_y;
    thrust::device_vector<double> velocity_x;
    thrust::device_vector<double> velocity_y; 
    thrust::device_vector<int> status;
    unsigned int pop_size;            // number of robot
    unsigned int dimension = 2; //set to 2
};

int main(void)
{
  unsigned int numRobots = 10;
  int numSteps = 10;    // understand it as "frames", how many steps in time

  RobotSystem test(numRobots);
  
  // Send new positions and velocities to GPU
  for (int i = 0; i < numRobots; i++)
    test.birth_robot();
  
  // Particle data lives in GPU now so we call the move on them few times!
  // This is great! As we don't have to be retrieving and re-sending, Thrust
  // functionality shines in this step. Great framework.
  for (int i = 0; i < numSteps; i++)
    test.advance_robot();
  std::cout << std::fixed;
	std::cout << std::setprecision(2);

  // This is gonna be an expensive way of accessing the positions, as for each
  // call to the ::operator[]() we are fetching the item from GPU to CPU. The 
  // right way would be to copy the device_vector into a host_vector like this:
  //    thrust::host_vector<double> host_vector = test.positions
  // That would do it paralelly, it would transfer all the items from the device
  // vector into the host_vector in a parallel way, but to keep it simply in the
  // code I will not be using it.

  return 0;
}

void RobotSystem::birth_robot()
{
  position_x.push_back(2.0);
  position_y.push_back(2.5);

  // velocity_x.push_back(2.0f * ((double)rand() / (double)RAND_MAX) - 1.0f);
  // velocity_y.push_back(2.0f * ((double)rand() / (double)RAND_MAX) - 1.0f);

  velocity_x.push_back(1.0);
  velocity_y.push_back(1.0);
  status.push_back(2);
}

void RobotSystem::advance_robot()
{
  // As we cannot send device vectors to the move (as device_vector is at
  // the end of the day a GPU structure abstraction in CPU) we have to get the
  // pointer in GPU memory in order for the move to know where to start 
  // reading the double arrays from.
  double* d_position_x =  thrust::raw_pointer_cast(&position_x[0]);
  double* d_position_y =  thrust::raw_pointer_cast(&position_y[0]);
  double* d_velocity_x = thrust::raw_pointer_cast(&velocity_x[0]);
  double* d_velocity_y = thrust::raw_pointer_cast(&velocity_y[0]);
  int* d_status = thrust::raw_pointer_cast(&status[0]);
  /* This is the way I structured my blocks and threads. I fixed the amount of
   * threads per block to 1024. So to get the amount of blocks we just get the
   * total number of elements in positions and divide it by 1024. We add one in
   * case the division leaves remainder.
   *
   * ┌──────────────────────grid─┬of─blocks─────────────────┬──────────
   * │     block_of_threads      │     block_of_threads     │  
   * │ ┌───┬───┬───────┬──────┐  │ ┌───┬───┬───────┬──────┐ │
   * │ │ 0 │ 1 │ [...] │ 16   │  │ │ 0 │ 1 │ [...] │ 16   │ │   ...
   * │ └───┴───┴───────┴──────┘  │ └───┴───┴───────┴──────┘ │
   * └───────────────────────────┴──────────────────────────┴──────────
   */

  dim3 blocksPerGrid(ceil(pop_size/16.0), 1, 1);
  dim3 threadsPerBlock(16, 1, 1);


  move_kernel<<<blocksPerGrid,threadsPerBlock>>>(d_position_x, d_position_y, d_velocity_x, d_velocity_y, d_status, pop_size);
  cudaDeviceSynchronize();
}

__device__ __host__ void print_pos(double* position_x, double* position_y, int i){
  printf("%f %f\n", position_x[i], position_y[i]);
}

/* pure move 
 * return value: true means entity collides on walls
 */
__device__ __host__ 
bool _move(unsigned int index, double *position_x, double *position_y, 
  double * velocity_x, double *velocity_y, double limit)
{
  bool res = false;
  double tmp = position_x[index] + velocity_x[index];
  if(tmp > limit || tmp < -limit){
        velocity_x[index] = -velocity_x[index];
        res = true;
  }
  tmp = position_y[index] + velocity_y[index];
  if(tmp > limit || tmp < -limit){
        velocity_y[index] = -velocity_y[index];
        res = true;
  }
  position_x[index] += velocity_x[index];
  position_y[index] += velocity_y[index];
  return res;
}

/* movement with respect to veclocity */
__device__ __host__ 
void move(unsigned int index, double *position_x, double *position_y, 
  double * velocity_x, double *velocity_y, int *status, double limit)
{
  if(status[index] == 1){
    status[index] = 2;
    return;
  }

  if(status[index] == 2){
    status[index] = 3;
  }

  _move(index, position_x, position_y, velocity_x, velocity_y, limit);
  
  /* update pos_next after real movement */
  // for(unsigned int i = 0; i < this->dimension; ++i){
  //   this->pos_next[i] = this->pos[i];
  // }
}