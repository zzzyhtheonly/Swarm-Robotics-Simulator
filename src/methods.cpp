#include <GL/gl.h>
#include <GL/freeglut.h>
#include <math.h>
#include <bits/stdc++.h>
#include <string>
#include "headers.h"

#ifdef GPU
double *g_pos_x = nullptr;
double *g_pos_y = nullptr;
double *g_pos_next_x = nullptr;
double *g_pos_next_y = nullptr;
char *g_bm = nullptr;

__device__ __host__ void print_pos(double*, double*, int);


__device__ __host__ void g_move(unsigned int, double *, double *, double * , double *, int *, double);
__device__ __host__ bool _g_move(unsigned int , double *, double *, double *, double *, double );

// This is the move that is launched from CPU and GPU runs it for each cell
__global__ void move_kernel(double *position_x, double *position_y, double *velocity_x, double *velocity_y, int *status, int pop_size)
{
    unsigned int index = blockDim.x * blockIdx.x + threadIdx.x;
    if (index >= pop_size) return;
    g_move(index, position_x, position_y, velocity_x, velocity_y, status, 1000.0);
    // position_x[index] += velocity_x[index];
    // position_y[index] += velocity_y[index];
    printf("%d, %f %f\n", index, position_x[index], position_y[index]);
    // positions[index] = index; // use this one for debugging the index
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
drawable::drawable(unsigned int dimension, double radius, double limit, unsigned int id)
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
			g_pos_x[id] = pos[i];
		} else if(i == 1){
			g_pos_y[id] = pos[i];
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
		log_file << this->id << "\t" << g_pos_x[this->id] << "\t" << g_pos_y[this->id] 
			<< "\t" << r << "\t" << g << "\t" << b << "\t" << std::endl;
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

objective::objective(unsigned int dimension, double radius, double limit, unsigned int id) : drawable(dimension, radius, limit, id) {
	/* Set my id */
	this->id = id;	
	/* Establish root of linked tree */
	this->link = new linked_tree(this, NULL, this);
};

/* 
 * dimension: fix to 2 at the moment
 * radius: fix to 20 at the moment
 * limit: playground dimension limit, only square allowed, assume that you want a 1000*1000 square then limit should be 1000
 */
individual::individual(unsigned int dimension, double radius, double limit, unsigned int mode, unsigned int id) : drawable(dimension, radius, limit, id)
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
			g_pos_x[id] = g_pos_next_x[id] = pos[i];
		} else if(i == 1){
			g_pos_y[id] = g_pos_next_y[id] = pos[i];
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

/* movement with respect to veclocity */
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
bool g_if_collision(unsigned int first, unsigned int second, bool first_use_pos_next, bool second_use_pos_next, double first_radius, double second_radius)
{
	double distance = 0;
	double x1, x2, y1, y2;

	x1 = first_use_pos_next ? g_pos_next_x[first] : g_pos_x[first];
	x2 = second_use_pos_next ? g_pos_next_x[second] : g_pos_x[second];
	y1 = first_use_pos_next ? g_pos_next_y[first] : g_pos_y[first];
	y2 = second_use_pos_next ? g_pos_next_y[second] : g_pos_y[second];
	
	double tmp = x1 - x2;
	distance += (tmp * tmp);
	tmp = y1 - y2;
	distance += (tmp * tmp);

	distance = sqrt(distance);

	return distance < first_radius + second_radius ? true : false;
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

void population::sense(double sense_dist)
{
	sense_objectives(sense_dist);
	sense_entities(sense_dist);
}

/* Is each entity within sensing distance of any of the objectives? */
void population::sense_objectives(double sense_dist)
{
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
}

/* Is each entity (who has not already sensed an objective, or is not already linked or
within a path) within sensing distance of at least one other linked entity? */
void population::sense_entities(double sense_dist)
{	
	for (unsigned int i = 0; i < this->pop_size; ++i){
		/* Continue to next entity if on objective */
		if (this->entities[i].status == ON_OBJ ||
			this->entities[i].status == LINK ||
			this->entities[i].status == PATH)
			continue;
		/* Is an entity within sensing distance of another entity
		who is in a linked tree? */
		for (unsigned int j = 0; j < this->pop_size; ++j) {
			if (i == j) continue;
			if (this->entities[i].if_sense(this->entities[j], sense_dist) && 
				this->entities[j].status == LINK) {
				//std::cout << "Entity " << i << " sensed entity " << j << std::endl;
				this->entities[i].status = SENSE;
			}
		}
	}
}

void population::decide(double sense_dist)
{
	decide_link_objective(sense_dist);
	decide_link_entity(sense_dist);	
}

/* An entity will always link with objectives */
void population::decide_link_objective(double sense_dist)
{
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
			std::cout << "Entity " << i << " is linking with objective " << obj_tmp->id << std::endl;
			this->entities[i].status = LINK;
			this->entities[i].velocity = vector<double>(this->dim, 0);
			this->entities[i].link = new linked_tree(obj_tmp->link->root, obj_tmp->link, &(this->entities[i]));
		}
	}
}

/* Entities will try to link with other linked entities if possible (free link or branch)
 If an entity can link with two linked entities from different linked_trees, a path is detected */
void population::decide_link_entity(double sense_dist)
{
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
			std::cout << "Entity " << i << " is linking with entity " << k_actual << std::endl;
			/* Entity is within sensing distance of a linked entity, so create another link */
			//another_tmp->link->free = false;
			this->entities[i].status = LINK;
			this->entities[i].velocity = vector<double>(this->dim, 0);
			this->entities[i].link = new linked_tree(another_tmp->link->root, another_tmp->link, &(this->entities[i]));
		}
	}	
}

/* test if collision exists otherwise update collision bitmap */
bool population::collision()
{
	bool res = false;

	/* TODO: make it supports GPU */
	for(unsigned int i = 0; i < this->pop_size; ++i){
		for(unsigned int j = i+1; j < this->pop_size; ++j){
			if(this->entities[i].if_collision(this->entities[j])){
				this->bm[i].bit = 1;
				this->bm[j].bit = 1;
				res = true;
			}
		}
	}

	return res;
}

/* customized collsion test only used after initialization */
bool population::init_collision()
{
	bool res = false;
	
#ifdef GPU
	for(unsigned int i = 0; i < this->pop_size + this->num_objs; ++i){
		for(unsigned int j = i+1; j < this->pop_size + this->num_objs; ++j){
			double radius_1 = i >= this->pop_size ? this->objectives[i-this->pop_size]->radius : this->entities[i].radius;
			double radius_2 = j >= this->pop_size ? this->objectives[j-this->pop_size]->radius : this->entities[j].radius;
			if(g_if_collision(i, j, false, false, radius_1, radius_2)){
				g_bm[i] = 1;
				g_bm[j] = 1;
				res = true;
			}
		}
	}
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

	// between objects
	for(unsigned int i = 0; i < this->num_objs; ++i){
		for(unsigned int j = i+1; j < this->num_objs; ++j){
			if(this->objectives[i]->if_collision(this->objectives[j])){
				this->bm[this->pop_size+i].bit = 1;
				this->bm[this->pop_size+j].bit = 1;
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

	return res;
	
}

/* adjuest velocity of each entity with respect to collision detection */
void population::adjustment()
{
	unsigned int retries = 0;

	/* retries 2 times because we only have 2 directions to go at the moment */
	while(collision() && retries < 2){
		/* TODO: GPU version */
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
			}
		}
	}

	/* collision still exists, stop the entities detected collision */
	while(collision()){
		/* TODO: GPU version */
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
	linked_tree *prev;
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
	std::cout << "FOUND A PATH" << std::endl;
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

/*
 * size: fix to 10 at the moment
 * others: same to the arguments of individual(...)
 */
population::population(unsigned int size, unsigned int dimension, double radius, double limit, unsigned int num_objectives, double objective_radius, unsigned int mode)
{
	this->pop_size = size;
	this->num_objs = num_objectives;
	this->dim = dimension;

#ifdef GPU
	gpu_uni_malloc((void **) &g_pos_x, (size+num_objectives) * sizeof(double));
	gpu_uni_malloc((void **) &g_pos_y, (size+num_objectives) * sizeof(double));
	gpu_uni_malloc((void **) &g_pos_next_x, size * sizeof(double));
	gpu_uni_malloc((void **) &g_pos_next_y, size * sizeof(double));
	gpu_uni_malloc((void **) &g_bm, (size+num_objectives) * sizeof(char));
#endif

	for(unsigned int i = 0; i < size; ++i){
		this->entities.push_back(individual(dimension, radius, limit, mode, i));
		this->bm.push_back(one_bit());
#ifdef GPU
		g_bm[i] = 0;
#endif
	}

	for(unsigned int i = 0; i < num_objectives; ++i) {
		objective *tmp = new objective(dimension, objective_radius, limit, size+i);
		this->objectives.push_back(tmp);
		this->bm.push_back(one_bit());
#ifdef GPU
		g_bm[i] = 0;
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
				this->entities[i] = individual(dimension, radius, limit, mode, i);
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
				this->objectives[i] = new objective(dimension, objective_radius, limit, size+i);
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
  status.push_back(2);
}

void population::advance_robot()
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
void g_move(unsigned int index, double *position_x, double *position_y, 
  double * velocity_x, double *velocity_y, int *status, double limit)
{
  if(status[index] == 1){
    status[index] = 2;
    return;
  }

  if(status[index] == 2){
    status[index] = 3;
  }

  _g_move(index, position_x, position_y, velocity_x, velocity_y, limit);
  
  /* update pos_next after real movement */
  // for(unsigned int i = 0; i < this->dimension; ++i){
  //   this->pos_next[i] = this->pos[i];
  // }
}
#endif
