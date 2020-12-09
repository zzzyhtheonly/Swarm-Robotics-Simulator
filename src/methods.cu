#include <GL/gl.h>
#include <GL/freeglut.h>
#include <math.h>
#include <bits/stdc++.h>

#include <algorithm>

#include <string>
#include "headers.h"

#ifdef GPU
#if 0 
double *g_pos_x = nullptr;
double *g_pos_y = nullptr;
double *g_pos_next_x = nullptr;
double *g_pos_next_y = nullptr;
char *g_bm = nullptr;
#endif

__device__ __host__ void print_pos(double*, double*, int);


__device__ __host__ void g_move(unsigned int, double *, double *, double *, 
							double *, double * , double *, int *, double );
__device__ __host__ bool _g_move(unsigned int , double *, double *, double *, double *, double );

// This is the move that is launched from CPU and GPU runs it for each cell
__global__ void move_kernel(double *position_x, double *position_y, double *position_next_x, 
	double *position_next_y, double *velocity_x, double *velocity_y, int *status, int pop_size, double limit)
{
    unsigned int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index >= pop_size) return;
    if(status[index] == G_STOP){
	    status[index] = G_READY;
	    return;
	  }

	  if(status[index] == G_READY){
	    status[index] = G_RUNNING;
	  }

	  double tmp = position_x[index] + velocity_x[index];
	  if(tmp > limit || tmp < -limit){
	        velocity_x[index] = -velocity_x[index];
	  }
	  tmp = position_y[index] + velocity_y[index];
	  if(tmp > limit || tmp < -limit){
	        velocity_y[index] = -velocity_y[index];
	  }
	  position_x[index] += velocity_x[index];
	  position_y[index] += velocity_y[index];
	  
	  /* update pos_next after real movement */
	  position_next_x[index] = position_x[index];
	  position_next_y[index] = position_y[index];
     // printf("%d, %f %f\n", index, position_x[index], position_y[index]);
}

__global__ void move_prediction_kernel(double *position_next_x, double *position_next_y, double *velocity_x, double *velocity_y, int pop_size, double limit)
{
	unsigned int index = blockDim.x * blockIdx.x + threadIdx.x;
	if (index >= pop_size) return;

	double tmp = position_next_x[index] + velocity_x[index];
	if(tmp > limit || tmp < -limit){
		velocity_x[index] = -velocity_x[index];
	}
	tmp = position_next_y[index] + velocity_y[index];
	if(tmp > limit || tmp < -limit){
	    velocity_y[index] = -velocity_y[index];
	}
	position_next_x[index] += velocity_x[index];
	position_next_y[index] += velocity_y[index];
}

__device__ __host__
g_linked_tree::g_linked_tree(int id, int r, int n)
{
	this->id = id;
	root = r;
	previous = -1;
	node = n;
	branch = 1;
	branch_dist = 0;
}

__device__ __host__
void g_form_path(unsigned int linked1, unsigned int linked2, unsigned int finder, 
	struct g_linked_tree* g_trees, int* g_trees_next, int *status, double* velocity_x, double* velocity_y, unsigned int total_size)
{
	unsigned int tmp = linked1;
	while (g_trees[tmp].node != g_trees[tmp].root) {
		status[g_trees[tmp].node] = G_PATH;
		tmp = g_trees[tmp].previous;
	}
	tmp = linked2;
	while (g_trees[tmp].node != g_trees[tmp].root) {
		status[g_trees[tmp].node] = G_PATH;
		tmp = g_trees[tmp].previous;
	}
	status[finder] = G_PATH;
	velocity_x[finder] = 0;
	velocity_y[finder] = 0;
	g_trees[finder].root = g_trees[linked1].root; 
	g_trees[finder].previous = linked1;
	g_trees[finder].node = finder;
	
	if (g_trees[finder].previous == -1) {
		g_trees[finder].branch = 1;
		g_trees[finder].branch_dist = 0;
	} else {
		g_trees_next[g_trees[finder].previous * total_size + g_trees[g_trees[finder].previous].end++] = finder;
		g_trees[finder].branch_dist = g_trees[g_trees[finder].previous].branch_dist+1;
		/* This link is a branch if it meets global variable branch_len */
		// branch_len is 5 fixed at GPU version
		if (g_trees[finder].branch_dist >= 5) {
			g_trees[finder].branch = 1;
			g_trees[finder].branch_dist = 0;
		} else {
			g_trees[finder].branch = 0;
		}
	}
	
	//printf("FOUND A PATH\n");
}
#endif

linked_tree::linked_tree(objective *r, linked_tree *p, drawable *n) {
	root = r;
	previous = p;
	node = n;
	if (p == NULL) {
		this->branch = true;
		this->branch_dist = 0;
	} else {
		previous->next.push_back(this);
		this->branch_dist = previous->branch_dist+1;
		/* This link is a branch if it meets global variable branch_len */
		if (branch_dist >= branch_len) {
			branch = true;
			branch_dist = 0;
		} else {
			branch = false;
		}
	}
}

/** Parent class for things that need to be displayed on screen **/
drawable::drawable(unsigned int dimension, double radius, double limit, unsigned int id, population& p)
{
	this->id = id;
	this->dimension = dimension;
	this->limit = limit;
	this->pos = vector<double>(dimension, 0);
	this->radius = radius;

	/* initialize coordinates randomly */
	random_device dev;
	mt19937 rng(dev());
	uniform_real_distribution<double> dist(0, limit);
	uniform_int_distribution<mt19937::result_type> sign(0, 1);

	for(unsigned int i = 0; i < dimension; ++i) {
		double val = dist(rng);
		this->pos[i] = sign(rng) ? val : -val;
#ifdef GPU
		if(i == 0){
			p.position_x[id] = pos[i];
		} else if(i == 1){
			p.position_y[id] = pos[i];
		}
#endif
	}
}

/* draw itself in OpenGL by filling with tiny triangles */
void drawable::draw(double r, double g, double b)
{
	unsigned int count = 20;
	GLfloat twicePi = 2.0f * M_PI;

	glBegin(GL_TRIANGLE_FAN);

		/* center */
#ifdef GPU
		//log_file << this->id << "\t" << g_pos_x[this->id] << "\t" << g_pos_y[this->id] 
			//<< "\t" << r << "\t" << g << "\t" << b << "\t" << std::endl;
		/*
		glVertex2f(g_pos_x[this->id], g_pos_y[this->id]);
		for(unsigned int i = 0; i <= count; ++i) {
			glVertex2f(g_pos_x[this->id] + (radius * cos(i * twicePi / count)), g_pos_y[this->id] + (radius * sin(i * twicePi / count)));
		}
		*/
#else
		glVertex2f(pos[0], pos[1]);

		for(unsigned int i = 0; i <= count; ++i) {
			glVertex2f(pos[0] + (radius * cos(i * twicePi / count)), pos[1] + (radius * sin(i * twicePi / count)));
		}
#endif
	glEnd();
}

objective::objective(unsigned int dimension, double radius, double limit, unsigned int id, population& p) : drawable(dimension, radius, limit, id, p) {
	/* Set my id */
	this->id = id;	
	/* Establish root of linked tree */
	this->link = new linked_tree(this, NULL, this);

#ifdef GPU
	this->g_link = id;
#endif
};

/* 
 * dimension: fix to 2 at the moment
 * radius: fix to 20 at the moment
 * limit: playground dimension limit, only square allowed, assume that you want a 1000*1000 square then limit should be 1000
 */
individual::individual(unsigned int dimension, double radius, double limit, unsigned int mode, unsigned int id, population& p) : drawable(dimension, radius, limit, id, p)
{
	this->id = id;
	this->dimension = dimension;
	this->limit = limit;
	this->pos = vector<double>(dimension, 0);
	this->pos_next = vector<double>(dimension, 0);
	this->velocity = vector<double>(dimension, 0);
	this->radius = radius;
	this->status = READY;
	this->mode = mode;
	this->id = id;


	/* initialize coordinates and velocities randomly */
	random_device dev;
	mt19937 rng(dev());
	uniform_real_distribution<double> dist(0, limit);
	uniform_int_distribution<mt19937::result_type> sign(0, 1);

	double fixed_velocity = ((double)limit / 10000.0);

	/* only works when dimension is 2 */
	/* not in use right noew */
	if(mode == LEFTMOST_INIT) {
		double val = dist(rng);
		this->pos[1] = this->pos_next[1] = sign(rng) ? val : -val;
		this->velocity[0] = fixed_velocity;
		this->pos[0] = this->pos_next[0] = -limit;
		this->velocity[1] = 0;

		return;
	}

	for(unsigned int i = 0; i < dimension; ++i) {
		double val = dist(rng);
		this->pos[i] = this->pos_next[i] = sign(rng) ? val : -val;
		this->velocity[i] = sign(rng) ? fixed_velocity : -fixed_velocity;
#ifdef GPU	
		if(i == 0){
			p.position_x[id] = pos[i];
			p.position_next_x[id] = pos[i];
			p.velocity_x[id] = velocity[i];
		} else if(i == 1){
			p.position_y[id] = pos[i];
			p.position_next_y[id] = pos[i];
			p.velocity_y[id] = velocity[i];
		}
#endif
	}
}

/* pure move 
 * return value: true means entity collides on walls
 */
bool individual::_move(vector<double>& pos)
{
	bool res = false;
	for(unsigned int i = 0; i < this->dimension; ++i){
		/* detect collision with walls */
		double tmp = pos[i] + this->velocity[i];	
		if(tmp > limit || tmp < -limit){
				velocity[i] = -velocity[i];
				res = true;
		}
		pos[i] += velocity[i];
	}

	return res;
}

/* movement with respect to velocity */
void individual::move()
{
	if(this->status == STOP){
		this->status = READY;
		return;
	}

	if(this->status == READY){
		this->status = RUNNING;
	}

	_move(this->pos);
	
	/* update pos_next after real movement */
	for(unsigned int i = 0; i < this->dimension; ++i){
		this->pos_next[i] = this->pos[i];
	}

}

/* predict movement with respect to velocity */
void individual::move_prediction()
{
	bool terminate = _move(this->pos_next);
	
	/* on leftmost mode, if entity reaches targets, terminate permanently */
	if(mode == LEFTMOST_INIT && terminate){
		this->status = TERMINATE;
		this->velocity = vector<double>(this->dimension, 0);
	}
}

/*
 * test base on pos_next because we make sure that there is no collision at initial
 * another: a specific entity to test collision
 */
#ifdef GPU
/* test collision base on given two indices */
__device__ __host__
bool g_if_collision(double* pos_x, double* pos_y, unsigned int first, unsigned int second, double first_radius, double second_radius, double sense_dist)
{
	double distance = 0;
	double x1, x2, y1, y2;
	
	x1 = pos_x[first];
	x2 = pos_x[second];
	y1 = pos_y[first];
	y2 = pos_y[second];
	
	double tmp = x1 - x2;
	distance += (tmp * tmp);
	tmp = y1 - y2;
	distance += (tmp * tmp);

	distance = sqrt(distance);

	return distance < first_radius + sense_dist + second_radius ? true : false;
}

bool population::g_if_collision(unsigned int first, unsigned int second, bool first_use_pos_next, bool second_use_pos_next, double first_radius, double second_radius)
{
	double distance = 0;
	double x1, x2, y1, y2;

	x1 = first_use_pos_next ? this->position_next_x[first] : this->position_x[first];
	x2 = second_use_pos_next ? this->position_next_x[second] : this->position_x[second];
	y1 = first_use_pos_next ? this->position_next_y[first] : this->position_y[first];
	y2 = second_use_pos_next ? this->position_next_y[second] : this->position_y[second];
	
	double tmp = x1 - x2;
	distance += (tmp * tmp);
	tmp = y1 - y2;
	distance += (tmp * tmp);

	distance = sqrt(distance);

	return distance < first_radius + second_radius ? true : false;
}

#if 0
__device__ __host__
void inner_collision_kernel(unsigned int pop_size, double radius, unsigned int i, char* res, double* pos_x, double* pos_y, char* d_bm)
{
	unsigned int j = blockDim.x * blockIdx.x + threadIdx.x;
    if (j <= i || j >= pop_size) return;
	
	*res = g_if_collision(pos_x, pos_y, i, j, radius, radius) ? 1 : *res;
	
}
#endif

__global__
void collision_kernel(unsigned int pop_size, double radius, char* res, double* pos_x, double* pos_y, char* d_bm)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size) return;

#if 0	
	dim3 blocksPerGrid(ceil((pop_size)/10.0), 1, 1);
	dim3 threadsPerBlock(10, 1, 1);

	inner_collision_kernel<<<blocksPerGrid,threadsPerBlock>>>(pop_size, radius, i, res, pos_x, pos_y, d_bm);
#endif
	
	for(unsigned int j = i+1; j < pop_size; ++j){
		if(g_if_collision(pos_x, pos_y, i, j, radius, radius, 0)){
			d_bm[i] = 1;
			d_bm[j] = 1;
			*res = 1;
		}
	}
	//printf("collision kernel: %f %f\n", pos_x[i], pos_y[i]);
}

__global__
void collision_diff_kernel(unsigned int pop_size, unsigned int obj_size, double radius1, double radius2, char* res, double* pos_x, double* pos_y, char* d_bm)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size) return;
	
	for(unsigned int j = 0; j < obj_size; ++j){
		if(g_if_collision(pos_x, pos_y, i, pop_size + j, radius1, radius2, 0)){
			d_bm[i] = 1;
			d_bm[pop_size+j] = 1;
			*res = 1;
		}
	}
	//printf("collision diff kernel: %d %f %f\n", i, pos_x[i], pos_y[i]);
}

__global__
void sense_kernel(unsigned int pop_size, double radius, 
	double* pos_x, double* pos_y, int *status, double sense_dist)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size || status[i] == G_LINK || status[i] == G_PATH || status[i] == G_ON_OBJ) return;
	
	for(unsigned int j = i+1; j < pop_size; ++j){
		if(g_if_collision(pos_x, pos_y, i, j, radius, radius, sense_dist) && status[j] == G_LINK){
			status[i] = G_SENSE;
			//printf("Entity %d sensed entity %d\n", i, j);
		}
	}
	
}

__global__
void sense_diff_kernel(unsigned int pop_size, unsigned int obj_size, double radius1, double radius2, 
	double* pos_x, double* pos_y, int *status, double sense_dist)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size || status[i] == G_LINK || status[i] == G_PATH) return;
	
	for(unsigned int j = 0; j < obj_size; ++j){
		if(g_if_collision(pos_x, pos_y, i, pop_size + j, radius1, radius2, sense_dist)){
			status[i] = G_ON_OBJ;
			//printf("Entity %d sensed objective %d\n", i, j);
		}
	}
	
}

__global__
void decide_kernel(unsigned int pop_size, double radius, struct g_linked_tree* g_trees, int* g_trees_next, 
	double* pos_x, double* pos_y, double* velocity_x, double* velocity_y, int *status, double sense_dist, unsigned int total_size)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size || status[i] != G_SENSE) return;
	
	int another = -1;
	int k_actual = -1;
	for(unsigned int j = i+1; j < pop_size; ++j){
		if(g_if_collision(pos_x, pos_y, i, j, radius, radius, sense_dist) && status[j] == G_LINK){
			if(another != -1 && g_trees[another].root != g_trees[j].root){
				g_form_path(another, j, i, g_trees, g_trees_next, status, velocity_x, velocity_y, total_size);
				break;
			}
			if(g_trees[j].end < 1 || g_trees[j].branch){
				another = j;
				k_actual = j;
			}
		}
	}
	
	//printf("Entity %d is trying to link with entity %d\n", i, k_actual);
	
	if(status[i] == G_PATH){
		return;
	}
	
	if(another == -1){
		status[i] = G_RUNNING;
		return;
	}
	
	status[i] = G_LINK;
	velocity_x[i] = 0;
	velocity_y[i] = 0;
	g_trees[i].id = i;
	g_trees[i].root = g_trees[another].root;
	g_trees[i].previous = another;
	g_trees[i].node = i;
	
	if (g_trees[i].previous == -1) {
		g_trees[i].branch = 1;
		g_trees[i].branch_dist = 0;
	} else {
		g_trees_next[g_trees[i].previous * total_size + g_trees[g_trees[i].previous].end++] = i;
		g_trees[i].branch_dist = g_trees[g_trees[i].previous].branch_dist+1;
		/* This link is a branch if it meets global variable branch_len */
		// branch_len is 5 fixed at GPU version
		if (g_trees[i].branch_dist >= 5) {
			g_trees[i].branch = 1;
			g_trees[i].branch_dist = 0;
		} else {
			g_trees[i].branch = 0;
		}
	}
	
	//printf("Entity %d is linking with entity %d\n", i, k_actual);
}

__global__
void decide_diff_kernel(unsigned int pop_size, unsigned int obj_size, double radius1, double radius2, struct g_linked_tree* g_trees, int* g_trees_next,
	double* pos_x, double* pos_y, double* velocity_x, double* velocity_y, int *status, double sense_dist, unsigned int total_size)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size || status[i] != G_ON_OBJ) return;
	
	int obj = -1;
	for(unsigned int j = 0; j < obj_size; ++j){
		if(g_if_collision(pos_x, pos_y, i, pop_size + j, radius1, radius2, sense_dist)){
			if(obj == -1){
				obj = j;
			}
		}
	}
	
	//printf("Entity %d is trying to link with objective %d\n", i, obj);
	
	if(obj == -1){
		status[i] = G_RUNNING;
		return;
	}
	
	status[i] = G_LINK;
	velocity_x[i] = 0;
	velocity_y[i] = 0;
	g_trees[i].id = i;
	g_trees[i].root = g_trees[obj].root;
	g_trees[i].previous = obj;
	g_trees[i].node = i;
	
	if (g_trees[i].previous == -1) {
		g_trees[i].branch = 1;
		g_trees[i].branch_dist = 0;
	} else {
		g_trees_next[g_trees[i].previous * total_size + g_trees[g_trees[i].previous].end++] = i;
		g_trees[i].branch_dist = g_trees[g_trees[i].previous].branch_dist+1;
		/* This link is a branch if it meets global variable branch_len */
		// branch_len is 5 fixed at GPU version
		if (g_trees[i].branch_dist >= 5) {
			g_trees[i].branch = 1;
			g_trees[i].branch_dist = 0;
		} else {
			g_trees[i].branch = 0;
		}
	}
	
	//printf("Entity %d is linking with objective %d\n", i, obj);
}
#endif

bool objective::if_collision(objective *another)
{
	double distance = 0;

	for(unsigned int i = 0; i < this->dimension; ++i){
		double tmp = this->pos[i] - another->pos[i];
		distance += (tmp * tmp);
	}

	distance = sqrt(distance);

	return distance < this->radius + another->radius ? true : false;
}

bool individual::if_collision(individual another)
{
	double distance = 0;

	for(unsigned int i = 0; i < this->dimension; ++i){
		double tmp = this->pos_next[i] - another.pos_next[i];
		distance += (tmp * tmp);
	}

	distance = sqrt(distance);

	return distance < this->radius + another.radius ? true : false;
}

bool individual::if_collision(objective *another)
{
	double distance = 0;

	for(unsigned int i = 0; i < this->dimension; ++i){
		double tmp = this->pos_next[i] - another->pos[i];
		distance += (tmp * tmp);
	}

	distance = sqrt(distance);

	return distance < this->radius + another->radius ? true : false;
}

bool individual::if_sense(individual another, double sense_dist) {
	double distance = 0;

	for(unsigned int i = 0; i < this->dimension; ++i){
		double tmp = this->pos_next[i] - another.pos[i];
		distance += (tmp * tmp);
	}

	distance = sqrt(distance);

	return distance < this->radius + sense_dist + another.radius ? true : false;	
}

bool individual::if_sense(objective *another, double sense_dist) {
	double distance = 0;

	for(unsigned int i = 0; i < this->dimension; ++i){
		double tmp = this->pos_next[i] - another->pos[i];
		distance += (tmp * tmp);
	}

	distance = sqrt(distance);

	return distance < this->radius + sense_dist + another->radius ? true : false;	
}

void individual::grid_coordinates(unsigned int &x, unsigned int &y, double limit, double cell) {
	x = (unsigned int)((this->pos[0]+limit)/cell);
	y = (unsigned int)((this->pos[1]+limit)/cell);	
}

void population::sense(double sense_dist)
{
	sense_objectives(sense_dist);
	sense_entities(sense_dist);
}

/* Is each entity within sensing distance of any of the objectives? */
void population::sense_objectives(double sense_dist)
{
#ifdef GPU
	double* d_position_x = thrust::raw_pointer_cast(&position_x[0]);
	double* d_position_y = thrust::raw_pointer_cast(&position_y[0]);
	int* d_status = thrust::raw_pointer_cast(&g_status[0]);

	dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
	dim3 threadsPerBlock(10, 1, 1);

	sense_diff_kernel<<<blocksPerGrid,threadsPerBlock>>>(pop_size, num_objs, this->entities[0].radius, this->objectives[0]->radius, 
		d_position_x, d_position_y, d_status, sense_dist);
	cudaDeviceSynchronize();

#else
	for (unsigned int i = 0; i < this->pop_size; ++i){
		if (this->entities[i].status == LINK || this->entities[i].status == PATH) continue;
		/* Is an entity on top of an objective? */
		for (unsigned int j = 0; j < this->num_objs; ++j) {
			if (this->entities[i].if_sense(this->objectives[j], sense_dist)) {
				//std::cout << "Entity " << i << " is on objective " << j << std::endl;
				this->entities[i].status = ON_OBJ;
			}
		}
	}
#endif
}

/* Is each entity (who has not already sensed an objective, or is not already linked or
within a path) within sensing distance of at least one other linked entity? */
void population::sense_entities(double sense_dist)
{	
#ifdef GPU
	double* d_position_x = thrust::raw_pointer_cast(&position_x[0]);
	double* d_position_y = thrust::raw_pointer_cast(&position_y[0]);
	int* d_status = thrust::raw_pointer_cast(&g_status[0]);

	dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
	dim3 threadsPerBlock(10, 1, 1);

	sense_kernel<<<blocksPerGrid,threadsPerBlock>>>(pop_size, this->entities[0].radius, d_position_x, d_position_y, d_status, sense_dist);
	cudaDeviceSynchronize();
#else
	unsigned int left_x, right_x, up_y, down_y;
	double l = this->dim_limit;
	double c = this->cell_size;
	double s = sense_dist + this->entities[0].radius*2;

	/* TODO: make it supports GPU */

	// Loop through each entity
	for(unsigned int i = 0; i < this->pop_size; ++i){
		/* Continue to next entity if in tree */

		if (this->entities[i].status == ON_OBJ ||
			this->entities[i].status == LINK ||
			this->entities[i].status == PATH)
			continue;

		// Each entity will look at their own cell, as well as adjacent cells (including diagonal)
		left_x = max(0, (int)(((this->entities[i].pos[0]+l)-s)/c));
		right_x = min((int)this->grid_size-1, (int)(((this->entities[i].pos[0]+l)+s)/c));
		down_y = max(0, (int)(((this->entities[i].pos[1]+l)-s)/c));
		up_y = min((int)this->grid_size-1, (int)(((this->entities[i].pos[1]+l)+s)/c));
		// Compare current entity to all other entities in current/adjacent cells
		for (unsigned int x = left_x; x <= right_x; x++) {
			for (unsigned int y = down_y; y <= up_y; y++) {
				// Iterate through all entities in comparison cell
				for (unsigned int k = 0; k < this->grid[x][y].size(); k++) {
					// j = index of comparison entity (index in this->entities)
					unsigned int j = this->grid[x][y][k];
					// Check if "sensing self"
					if (i == j) continue;
					//cout << i << " | " << j << endl;
					if (this->entities[i].if_sense(this->entities[j], sense_dist) && 
						this->entities[j].status == LINK) {
						//std::cout << "Entity " << i << " sensed entity " << j << std::endl;
						this->entities[i].status = SENSE;
					}
				}
			}
		}
	}
#endif
}

void population::decide(double sense_dist)
{
	decide_link_objective(sense_dist);
	decide_link_entity(sense_dist);	
}

/* An entity will always link with objectives */
void population::decide_link_objective(double sense_dist)
{
#ifdef GPU
	double* d_position_x = thrust::raw_pointer_cast(&position_x[0]);
	double* d_position_y = thrust::raw_pointer_cast(&position_y[0]);
	double* d_velocity_x = thrust::raw_pointer_cast(&velocity_x[0]);
	double* d_velocity_y = thrust::raw_pointer_cast(&velocity_y[0]);
	int* d_status = thrust::raw_pointer_cast(&g_status[0]);
	struct g_linked_tree* d_linked_tree = thrust::raw_pointer_cast(&g_trees[0]);
	int* g_next = thrust::raw_pointer_cast(&g_trees_next[0]);

	dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
	dim3 threadsPerBlock(10, 1, 1);

	decide_diff_kernel<<<blocksPerGrid,threadsPerBlock>>>(pop_size, num_objs, this->entities[0].radius, this->objectives[0]->radius, 
		d_linked_tree, g_next, d_position_x, d_position_y, d_velocity_x, d_velocity_y, d_status, sense_dist, total_size);
	cudaDeviceSynchronize();
#else
	for (unsigned int i = 0; i < this->pop_size; ++i){
		/* If the entity sensed that it was on an objective */
		if (this->entities[i].status == ON_OBJ) {
			/* Find objective that this entity is on top of */
			objective *obj_tmp = NULL;
			for (unsigned int j = 0; j < this->num_objs; ++j) {
				if (this->entities[i].if_sense(this->objectives[j], sense_dist)) {
					if (obj_tmp == NULL) {
						obj_tmp = this->objectives[j];
					} else {
						/* Single link between objectives would go here */
					}
				}
			}
			/* not sure if this can happen, but return to running if so */
			if (obj_tmp == NULL) {
				this->entities[i].status = RUNNING;
				continue;
			}
			/* Otherwise this entity is now in a linked_tree */
			//std::cout << "Entity " << i << " is linking with objective " << obj_tmp->id << std::endl;
			this->entities[i].status = LINK;
			this->entities[i].velocity = vector<double>(this->dim, 0);
			this->entities[i].link = new linked_tree(obj_tmp->link->root, obj_tmp->link, &(this->entities[i]));
		}
	}
#endif
}

/* Entities will try to link with other linked entities if possible (free link or branch)
 If an entity can link with two linked entities from different linked_trees, a path is detected */
void population::decide_link_entity(double sense_dist)
{
#ifdef GPU
	double* d_position_x = thrust::raw_pointer_cast(&position_x[0]);
	double* d_position_y = thrust::raw_pointer_cast(&position_y[0]);
	double* d_velocity_x = thrust::raw_pointer_cast(&velocity_x[0]);
	double* d_velocity_y = thrust::raw_pointer_cast(&velocity_y[0]);
	int* d_status = thrust::raw_pointer_cast(&g_status[0]);
	struct g_linked_tree* d_linked_tree = thrust::raw_pointer_cast(&g_trees[0]);
	int* g_next = thrust::raw_pointer_cast(&g_trees_next[0]);

	dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
	dim3 threadsPerBlock(10, 1, 1);

	decide_kernel<<<blocksPerGrid,threadsPerBlock>>>(pop_size, this->entities[0].radius,
		d_linked_tree, g_next, d_position_x, d_position_y, d_velocity_x, d_velocity_y, d_status, sense_dist, total_size);
	cudaDeviceSynchronize();
#else
	for (unsigned int i = 0; i < this->pop_size; ++i){
		if (this->entities[i].status == SENSE) {
			/* Find the first linked entity that is also free or able to branch */
			//std::cout << "Entity " << i << " is deciding what to do after sensing another linked entity" << std::endl;
			individual *another_tmp = NULL;
			unsigned int k_actual = -1;
			for (unsigned int k = 0; k < this->pop_size; ++k) {
				if (i == k) continue;
				if (this->entities[i].if_sense(this->entities[k], sense_dist) &&
					this->entities[k].status == LINK) {
					if (another_tmp != NULL && another_tmp->link->root != this->entities[k].link->root) {
						form_path(another_tmp, &(this->entities[k]), &(this->entities[i]));
						break;
					}
					if (this->entities[k].link->next.size() < 1 || this->entities[k].link->branch) {
						another_tmp = &(this->entities[k]);
						k_actual = k;
					}
				}
			}
			if (this->entities[i].status == PATH) {
				continue;
			}
			/* This can happen if two entities sense the same linked entity (one will link with it before the other) */
			if (another_tmp == NULL) {
				this->entities[i].status = RUNNING;
				continue;
			}
			//std::cout << "Entity " << i << " is linking with entity " << k_actual << std::endl;
			/* Entity is within sensing distance of a linked entity, so create another link */
			//another_tmp->link->free = false;
			this->entities[i].status = LINK;
			this->entities[i].velocity = vector<double>(this->dim, 0);
			this->entities[i].link = new linked_tree(another_tmp->link->root, another_tmp->link, &(this->entities[i]));
		}
	}
#endif	
}

/* test if collision exists otherwise update collision bitmap 
	comparisons are only made to nearby entities */

bool population::collision()
{

	bool res = false;
	unsigned int left_x, right_x, up_y, down_y;
	double l = this->dim_limit;
	double c = this->cell_size;
	double r = this->entities[0].radius*2;

	/* TODO: make it supports GPU */

#ifdef GPU
	double* d_position_x =  thrust::raw_pointer_cast(&position_next_x[0]);
	double* d_position_y =  thrust::raw_pointer_cast(&position_next_y[0]);
	char* d_bm =  thrust::raw_pointer_cast(&g_bm[0]);
	char* d_res;
	cudaError_t err = cudaMallocManaged((void **) &d_res, 1);
	if(err != cudaSuccess)
	{
		printf("%s in %s at line %d\n", cudaGetErrorString(err), __FILE__, __LINE__);
		exit(EXIT_FAILURE);
	}
	*d_res = 0;

	dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
	dim3 threadsPerBlock(10, 1, 1);

	collision_kernel<<<blocksPerGrid,threadsPerBlock>>>(this->pop_size, this->entities[0].radius, d_res, d_position_x, d_position_y, d_bm);
	cudaDeviceSynchronize();
	res = *d_res ? true : res;
	cudaFree(d_res);

#else
	// Loop through each entity
	for(unsigned int i = 0; i < this->pop_size; ++i){
		// Each entity will look at their own cell, as well as adjacent cells (including diagonal)
		left_x = max(0, (int)((this->entities[i].pos[0]+l-r)/c));
		right_x = min((int)this->grid_size-1, (int)((this->entities[i].pos[0]+l+r)/c));
		down_y = max(0, (int)((this->entities[i].pos[1]+l-r)/c));
		up_y = min((int)this->grid_size-1, (int)((this->entities[i].pos[1]+l+r)/c));
		// Compare current entity to all other entities in current/adjacent cells
		for (unsigned int x = left_x; x <= right_x; x++) {
			for (unsigned int y = down_y; y <= up_y; y++) {
				// Iterate through all entities in comparison cell
				for (unsigned int k = 0; k < this->grid[x][y].size(); k++) {
					// j = index of comparison entity (index in this->entities)
					unsigned int j = this->grid[x][y][k];
					// Check if "comparing to self"
					if (i == j) continue;
					// If colliding with another, then do collision behavior
					if (this->entities[i].if_collision(this->entities[j])) {
						this->bm[i].bit=1;
						this->bm[j].bit=1;
						res = true;
					}
				}
			}
		}
	}
#endif

	return res;
}

/* customized collsion test only used after initialization */
bool population::init_collision()
{
	bool res = false;
	
#ifdef GPU

	double* d_position_x =  thrust::raw_pointer_cast(&position_x[0]);
	double* d_position_y =  thrust::raw_pointer_cast(&position_y[0]);
	char* d_bm =  thrust::raw_pointer_cast(&g_bm[0]);
	char* d_res;
	cudaError_t err = cudaMallocManaged((void **) &d_res, 1);
	if(err != cudaSuccess)
	{
		printf("%s in %s at line %d\n", cudaGetErrorString(err), __FILE__, __LINE__);
		exit(EXIT_FAILURE);
	}
	*d_res = 0;
	cudaStream_t streams[2];
	cudaStreamCreate(&streams[0]);
	cudaStreamCreate(&streams[1]);

	dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
	dim3 threadsPerBlock(10, 1, 1);

	// between entities
	collision_kernel<<<blocksPerGrid,threadsPerBlock, 0, streams[0]>>>(this->pop_size, this->entities[0].radius, d_res, d_position_x, d_position_y, d_bm);

	// between entities and objects
	collision_diff_kernel<<<blocksPerGrid,threadsPerBlock, 0, streams[1]>>>(this->pop_size, this->num_objs, this->entities[0].radius, this->objectives[0]->radius, d_res, d_position_x, d_position_y, d_bm);
	
	cudaDeviceSynchronize();
	
	res = *d_res ? true : res;
	cudaFree(d_res);

#else
	// between entities
	for(unsigned int i = 0; i < this->pop_size; ++i){
		for(unsigned int j = i+1; j < this->pop_size; ++j){
			if(this->entities[i].if_collision(this->entities[j])){
				this->bm[i].bit = 1;
				this->bm[j].bit = 1;
				res = true;
			}
		}
	}

	// between entities and objects
	for(unsigned int i = 0; i < this->pop_size; ++i){
		for(unsigned int j = 0; j < this->num_objs; ++j){
			if(this->entities[i].if_collision(this->objectives[j])){
				this->bm[i].bit = 1;
				this->bm[this->pop_size+j].bit = 1;
				res = true;
			}
		}
	}
#endif

	// between objects
	for(unsigned int i = 0; i < this->num_objs; ++i){
		for(unsigned int j = i+1; j < this->num_objs; ++j){
			if(this->objectives[i]->if_collision(this->objectives[j])){
				this->bm[this->pop_size+i].bit = 1;
				this->bm[this->pop_size+j].bit = 1;
#ifdef GPU
				g_bm[this->pop_size+i] = 1;
				g_bm[this->pop_size+j] = 1;
#endif
				res = true;
			}
		}
	}

	return res;
	
}

__global__
void first_adjustment_kernel(unsigned int pop_size, double* pos_x, double* pos_y,
	double* pos_next_x, double* pos_next_y, double* velocity_x, double* velocity_y, char* d_bm, double limit)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size || d_bm[i] == 0) return;
	
	d_bm[i] = 0;
	velocity_x[i] = -velocity_x[i];
	velocity_y[i] = -velocity_y[i];
	pos_next_x[i] = pos_x[i];
	pos_next_y[i] = pos_y[i];
	
	double tmp = pos_next_x[i] + velocity_x[i];
	if(tmp > limit || tmp < -limit){
		velocity_x[i] = -velocity_x[i];
	}
	tmp = pos_next_y[i] + velocity_y[i];
	if(tmp > limit || tmp < -limit){
	    velocity_y[i] = -velocity_y[i];
	}
	pos_next_x[i] += velocity_x[i];
	pos_next_y[i] += velocity_y[i];
}

__global__
void second_adjustment_kernel(unsigned int pop_size, double* pos_x, double* pos_y,
	double* pos_next_x, double* pos_next_y, int* d_status, char* d_bm)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size || d_bm[i] == 0) return;
	
	d_bm[i] = 0;
	pos_next_x[i] = pos_x[i];
	pos_next_y[i] = pos_y[i];
	d_status[i] = G_STOP;
}

/* adjuest velocity of each entity with respect to collision detection */
void population::adjustment()
{
	unsigned int retries = 0;

	/* retries 2 times because we only have 2 directions to go at the moment */
	while(collision() && retries++ < 2){
#ifdef GPU
		double* d_position_x = thrust::raw_pointer_cast(&position_x[0]);
		double* d_position_y = thrust::raw_pointer_cast(&position_y[0]);
		double* d_position_next_x = thrust::raw_pointer_cast(&position_next_x[0]);
		double* d_position_next_y = thrust::raw_pointer_cast(&position_next_y[0]);
		double* d_velocity_x = thrust::raw_pointer_cast(&velocity_x[0]);
		double* d_velocity_y = thrust::raw_pointer_cast(&velocity_y[0]);

		char* d_bm =  thrust::raw_pointer_cast(&g_bm[0]);

		dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
		dim3 threadsPerBlock(10, 1, 1);

		first_adjustment_kernel<<<blocksPerGrid,threadsPerBlock>>>(this->pop_size, d_position_x, d_position_y, 
			d_position_next_x, d_position_next_y, d_velocity_x, d_velocity_y, d_bm, this->limit);
		cudaDeviceSynchronize();

#else
		for(unsigned int i = 0; i < this->pop_size; ++i){
			if(this->bm[i].bit){
				this->bm[i].bit = 0;
				/* simply reverse the direction at the moment */
				/* also update pos_next */
				/* TODO: should be calculate by genetic algorithm */
				for(unsigned int j = 0; j < this->entities[i].dimension; ++j){
					this->entities[i].velocity[j] = -this->entities[i].velocity[j];
					this->entities[i].pos_next[j] = this->entities[i].pos[j];
				}

				this->entities[i].move_prediction();
				
				/* FIXME: copy host back to device, shouldn't do that for final product */
			}
		}
#endif
	}

	/* collision still exists, stop the entities detected collision */
	retries = 0;
	while(collision() && retries < 2){
#ifdef GPU
		double* d_position_x = thrust::raw_pointer_cast(&position_x[0]);
		double* d_position_y = thrust::raw_pointer_cast(&position_y[0]);
		double* d_position_next_x = thrust::raw_pointer_cast(&position_next_x[0]);
		double* d_position_next_y = thrust::raw_pointer_cast(&position_next_y[0]);
		int* d_g_status = thrust::raw_pointer_cast(&g_status[0]);

		char* d_bm =  thrust::raw_pointer_cast(&g_bm[0]);

		dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
		dim3 threadsPerBlock(10, 1, 1);

		second_adjustment_kernel<<<blocksPerGrid,threadsPerBlock>>>(this->pop_size, d_position_x, d_position_y, 
			d_position_next_x, d_position_next_y, d_g_status, d_bm);
		cudaDeviceSynchronize();
#else
		for(unsigned int i = 0; i < this->pop_size; ++i){
			if(this->bm[i].bit){
				this->bm[i].bit = 0;
				this->entities[i].status = STOP;
				/* reset pos_next */
				for(unsigned int j = 0; j < this->entities[i].dimension; ++j){
					this->entities[i].pos_next[j] = this->entities[i].pos[j];
				}
			}
		}
#endif
	}
	
	if(retries == 2){
		std:: cout << "No room for that much(big) entities! "
			   << "Program exit at this point because collision between entities could not be solved "
			   << "Please revise your arguments by "
			   << "decresing the [population size], [radius of entity] "
			   << "or incresing the [playground dimension]" 
			   << endl;
		exit(1);
	}
}

/* check if all entities terminate */
bool population::terminate()
{
	bool res = true;

	for(auto e : entities){
		if(e.status != TERMINATE){
			res = false;
			break;
		}
	}

	return res;
}

void population::form_path(individual *linked1, individual *linked2, individual *finder) {

	//linked_tree *prev;

	linked_tree *tmp = linked1->link;
	while (tmp->node != tmp->root) {
		((individual *)tmp->node)->status = PATH;
		tmp = tmp->previous;
	}
	tmp = linked2->link;
	while (tmp->node != tmp->root) {
		((individual *)tmp->node)->status = PATH;
		tmp = tmp->previous;
	}
	finder->status = PATH;
	finder->velocity = vector<double>(this->dim, 0);
	finder->link = new linked_tree(linked1->link->root, linked1->link, finder);
	//std::cout << "FOUND A PATH" << std::endl;
}

void population::init_grid(double radius, double dimension_limit) {
	this->cell_size = 1;
	while (cell_size < (radius*2)) { cell_size *= 2; }
	this->grid_size = ((dimension_limit*2)/cell_size)+1;

	this->grid = new vector<unsigned int> *[grid_size];
	for (unsigned int i = 0; i < grid_size; i++) {
		this->grid[i] = new vector<unsigned int> [grid_size];
		for (unsigned int j = 0; j < grid_size; j++) {
			this->grid[i][j] = vector<unsigned int>();
		}
	}
	//std::cout << "Grid is " << grid_size << " by " << grid_size << " with cell size " << cell_size << std::endl;
}

void population::clear_grid() {
	for (unsigned int i = 0; i < grid_size; i++) {
		for (unsigned int j = 0; j < grid_size; j++) {
			this->grid[i][j].clear();
		}
	}
}

void population::assign_to_grid() {
	unsigned int grid_x,grid_y;
	for (unsigned int i = 0; i < this->pop_size; i++) {
		// grid_x, grid_y are being passed by reference
		this->entities[i].grid_coordinates(grid_x, grid_y, this->dim_limit, this->cell_size);	
		this->grid[grid_x][grid_y].push_back(this->entities[i].id);
	}
}


#ifdef GPU
void gpu_uni_malloc(void **buf, size_t size)
{
	cudaError_t err = cudaMallocManaged(buf, size);
	if(err != cudaSuccess){
		printf("%s in %s at line %d\n", cudaGetErrorString(err), __FILE__, __LINE__);
		exit(EXIT_FAILURE);
	}
}
#endif

__global__ void draw_kernel(double *position_x, double *position_y, int *status, int pop_size, int num_objs)
{
	unsigned int index = blockDim.x * blockIdx.x + threadIdx.x;
	if (index >= pop_size+num_objs) return;
	else if (index >= pop_size) {
		printf("%d\t%f\t%f\t1.\t0.\t0.\n",index, position_x[index], position_y[index]);
	}
	else {
		if (status[index] == G_LINK)
			printf("%d\t%f\t%f\t0.\t1.\t0.\n",index, position_x[index], position_y[index]);
		else if (status[index] == G_PATH)
			printf("%d\t%f\t%f\t.5\t0.\t.5\n",index, position_x[index], position_y[index]);
		else
			printf("%d\t%f\t%f\t0.\t0.\t1.\n",index, position_x[index], position_y[index]);
	}

}

void population::draw() {
	double* d_position_x = thrust::raw_pointer_cast(&position_x[0]);
	double* d_position_y = thrust::raw_pointer_cast(&position_y[0]);
	double* d_velocity_x = thrust::raw_pointer_cast(&velocity_x[0]);
	double* d_velocity_y = thrust::raw_pointer_cast(&velocity_y[0]);
	int* d_g_status = thrust::raw_pointer_cast(&g_status[0]);


	dim3 blocksPerGrid(ceil((pop_size+num_objs)/16.0), 1, 1);
	dim3 threadsPerBlock(16, 1, 1);


	draw_kernel<<<blocksPerGrid,threadsPerBlock>>>(d_position_x, d_position_y, d_g_status, pop_size, num_objs);
	cudaDeviceSynchronize();
}

/*
 * size: fix to 10 at the moment
 * others: same to the arguments of individual(...)
 */
population::population(unsigned int size, unsigned int dimension, double radius, double limit, unsigned int num_objectives, double objective_radius, unsigned int mode)
{
	this->pop_size = size;
	this->num_objs = num_objectives;
	this->dim = dimension;
	this->dim_limit = limit;


#ifdef GPU
	this->total_size = size + num_objectives;
	this->position_x = thrust::device_vector<double>(total_size, 0);
	this->position_y = thrust::device_vector<double>(total_size, 0);
	this->position_next_x = thrust::device_vector<double>(size, 0);
	this->position_next_y = thrust::device_vector<double>(size, 0);
	this->velocity_x = thrust::device_vector<double>(size, 0);
	this->velocity_y = thrust::device_vector<double>(size, 0);

	this->g_status = thrust::device_vector<double>(size, 2);
	this->g_bm = thrust::device_vector<char>(total_size, 0);
	this->limit = limit;
	
	this->g_trees = thrust::device_vector<struct g_linked_tree>(total_size, g_linked_tree(-1, -1, -1));
	this->g_trees_next = thrust::device_vector<int>(total_size*total_size, -1);
	
#if 0
	gpu_uni_malloc((void **) &g_pos_x, (size+num_objectives) * sizeof(double));
	gpu_uni_malloc((void **) &g_pos_y, (size+num_objectives) * sizeof(double));
	gpu_uni_malloc((void **) &g_pos_next_x, size * sizeof(double));
	gpu_uni_malloc((void **) &g_pos_next_y, size * sizeof(double));
	gpu_uni_malloc((void **) &g_bm, (size+num_objectives) * sizeof(char));
#endif
	
#endif

	for(unsigned int i = 0; i < size; ++i){
		this->entities.push_back(individual(dimension, radius, limit, mode, i, *this));
		this->bm.push_back(one_bit());
#ifdef GPU
		g_bm[i] = 0;
#endif
	}

	for(unsigned int i = 0; i < num_objectives; ++i) {
		objective *tmp = new objective(dimension, objective_radius, limit, size+i, *this);
		this->objectives.push_back(tmp);
		this->bm.push_back(one_bit());
#ifdef GPU
		g_bm[size+i] = 0;
		g_trees.push_back(g_linked_tree(size+i, size+i, size+i));
#endif
	}

	unsigned int retries = 0;
	/* make sure the population is initialized with no collision, give a retry limitation to prevent forever loop */
	while(init_collision() && retries++ < 99){
		/* TODO: GPU version */
		for(unsigned int i = 0; i < size; ++i){
#ifdef GPU
			if(g_bm[i]){
				g_bm[i] = 0;
#else
			if(this->bm[i].bit){
				this->bm[i].bit = 0;
#endif
				this->entities[i] = individual(dimension, radius, limit, mode, i, *this);
			}
		}

		for(unsigned int i = 0; i < num_objectives; ++i){
#ifdef GPU
			if(g_bm[size+i]){
				g_bm[size+i] = 0;
#else
			if(this->bm[size+i].bit){
				this->bm[size+i].bit = 0;
#endif
				/* FIXME: memory leak here, we need a destructor to make it works properly */
				this->objectives[i] = new objective(dimension, objective_radius, limit, size+i, *this);
			}
		}
	}

	if(retries == 100){
		std:: cout << "No room for that much(big) entities! "
			   << "Please revise your arguments by "
			   << "decresing the [population size], [radius of entity] "
			   << "or incresing the [playground dimension]" 
			   << endl;
		exit(1);
	}

	this->init_grid(radius, limit);
	this->clear_grid();
	this->assign_to_grid();

}


#ifdef GPU
void population::birth_robot()
{
  position_x.push_back(2.0);
  position_y.push_back(2.5);

  // velocity_x.push_back(2.0f * ((double)rand() / (double)RAND_MAX) - 1.0f);
  // velocity_y.push_back(2.0f * ((double)rand() / (double)RAND_MAX) - 1.0f);

  velocity_x.push_back(1.0);
  velocity_y.push_back(1.0);

  g_status.push_back(2);
}

void population::advance_robot()
{
  // As we cannot send device vectors to the move (as device_vector is at
  // the end of the day a GPU structure abstraction in CPU) we have to get the
  // pointer in GPU memory in order for the move to know where to start 
  // reading the double arrays from.

  double* d_position_x = thrust::raw_pointer_cast(&position_x[0]);
  double* d_position_y = thrust::raw_pointer_cast(&position_y[0]);
  double* d_position_next_x = thrust::raw_pointer_cast(&position_next_x[0]);
  double* d_position_next_y = thrust::raw_pointer_cast(&position_next_y[0]);
  double* d_velocity_x = thrust::raw_pointer_cast(&velocity_x[0]);
  double* d_velocity_y = thrust::raw_pointer_cast(&velocity_y[0]);
  int* d_g_status = thrust::raw_pointer_cast(&g_status[0]);


  dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
  dim3 threadsPerBlock(10, 1, 1);


  move_kernel<<<blocksPerGrid,threadsPerBlock>>>(d_position_x, d_position_y, d_position_next_x,
   d_position_next_y, d_velocity_x, d_velocity_y, d_g_status, pop_size, this->limit);
  cudaDeviceSynchronize();

}

void population::predict_robot()
{
  double* d_position_next_x = thrust::raw_pointer_cast(&position_next_x[0]);
  double* d_position_next_y = thrust::raw_pointer_cast(&position_next_y[0]);
  double* d_velocity_x = thrust::raw_pointer_cast(&velocity_x[0]);
  double* d_velocity_y = thrust::raw_pointer_cast(&velocity_y[0]);

  dim3 blocksPerGrid(ceil(pop_size/10.0), 1, 1);
  dim3 threadsPerBlock(10, 1, 1);

  move_prediction_kernel<<<blocksPerGrid,threadsPerBlock>>>(d_position_next_x, d_position_next_y, d_velocity_x, d_velocity_y, pop_size, this->limit);
  cudaDeviceSynchronize();
}

__device__ __host__ void print_pos(double* position_x, double* position_y, int i){
  printf("%f %f\n", position_x[i], position_y[i]);
}

/* pure move 
 * return value: true means entity collides on walls
 */
__device__ __host__ 
bool _g_move(unsigned int index, double *position_x, double *position_y, 
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
void g_move(unsigned int index, double *position_x, double *position_y, double *position_next_x, 
	double *position_next_y, double * velocity_x, double *velocity_y, int *status, double limit)
{
  if(status[index] == 1){
    status[index] = 2;
    return;
  }

  if(status[index] == 2){
    status[index] = 3;
  }

  double tmp = position_x[index] + velocity_x[index];
  if(tmp > limit || tmp < -limit){
        velocity_x[index] = -velocity_x[index];
  }
  tmp = position_y[index] + velocity_y[index];
  if(tmp > limit || tmp < -limit){
        velocity_y[index] = -velocity_y[index];
  }
  position_x[index] += velocity_x[index];
  position_y[index] += velocity_y[index];
  
  /* update pos_next after real movement */
  position_next_x[index] = position_x[index];
  position_next_y[index] = position_y[index];
}
#endif
