#include <vector>
#include <list>

#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <fstream>

#ifdef GPU
#include <cuda.h>
#include <cuda_runtime_api.h>
#include <thrust/device_vector.h>
#endif

#define RANDOM_INIT 0
#define LEFTMOST_INIT 1

using namespace::std;

/* refine every GPU-friendly variables as global pointers for GPU version*/
#ifdef GPU
/* should be pop_size + num_objs, could be found in class population, only support 2 dimension at the moment */
extern double *g_pos_x;
extern double *g_pos_y;
/* should be pop_size */
extern double *g_pos_next_x;
extern double *g_pos_next_y;
/* should be pop_size + num_objs, could be found in class population */
extern char *g_bm;
extern ofstream log_file;
#endif

class drawable;
class objective;
class individual;
class population;

extern double radius;
extern double objective_radius;

/* define a one bit data structure for bitmap */
typedef struct one_bit
{
	unsigned bit:1;

	one_bit() { bit = 0; }
}one_bit;

/* states describe a robot's status */
enum states
{
	TERMINATE,
	STOP,
	READY,
	RUNNING,
	ON_OBJ,
	LINK,
	SENSE,
	PATH,
	BRANCH,
};

/* Made this a global variable so main.cpp can set it at runtime. Maybe better way to do this? */
extern unsigned int branch_len;

/* Linked list that can branch after a defined number of nodes */
class linked_tree
{
public:
	/* Objective that this linked tree originates from */
	objective *root;
	/* Previous link which is closer in the tree to the root */
	linked_tree *previous;
	/* Links that come after this one */
	vector<linked_tree *> next;
	/* What is at this link in the linked tree */
	drawable *node;
	/* Distance from last branch */
	unsigned int branch_dist;
	/* Indicates if the tree can branch at this link */
	bool branch = false;

	linked_tree() = delete;
	linked_tree(objective *r, linked_tree *p, drawable *n);
};

class drawable
{
public:
	/* unqiue id for each entity */
	unsigned int id;

	/* dimension of our space, fixed to 2 at this moment */
	unsigned int dimension;

	/* coordinate limitation */
	double limit;

	/* position of each entity, size should be fixed to 2 at this moment, we dont use pair for efficiency here because it may be extended to 3 dimension one day */
	vector<double> pos;

	/* radius for entity since it is a circle */
	double radius;

	/* Assume any drawable can be placed in a linked tree, for now */
	linked_tree *link;

	/* construct functions */
	drawable() = delete;
	drawable(unsigned int dimension, double radius, double limit, unsigned int id, population& p);

	/* draw circle */
	void draw(double r, double g, double b);
};

/* Used to differentiate objectives from other drawables
   Can expand this later */
class objective : public drawable {
public:
	/* collision detection between objectives, only could happen after initialization */
	bool if_collision(objective *another);

	/* construct functions */
	objective() = delete;
	objective(unsigned int dimension, double radius, double limit, unsigned int id, population& p);

};

/* class represents a single robot entity */
class individual : public drawable
{
public:

	unsigned int id = -1;

	/* current status */
	states status;

	/* simulator mode */
	unsigned int mode;
	
	/* next position of each entity with respect to the current velocity, it is used to adjust velocity in order to avoid collision */
	vector<double> pos_next;

	/* velocity for each direction, should coincide with dimension of pos */
	vector<double> velocity;

	/* construct functions */
	individual() = delete;
	individual(unsigned int dimension, double radius, double limit, unsigned int mode, unsigned int id, population& p);

	/* no checks move, return true if there is collisions with walls */
	bool _move(vector<double>&);

	/* movement with respect to velocity, affects pos */
	void move();

	/* movement prediction, affects pos_next */
	void move_prediction();

	/* collision detection between this and given entity */
	bool if_collision(individual another);
	bool if_collision(objective *another);

	/* Sensing detection between this and given entity */
	bool if_sense(individual another, double sense_dist);
	bool if_sense(objective *another, double sense_dist);

	void grid_coordinates(unsigned int &x, unsigned int &y, double limit, double cell);

	/* TODO: for genetic algorithm to calculate fitness */
	void calc_fitness() {}
};

/* class represents  */
class population
{
public:

#ifdef GPU
    thrust::device_vector<double> position_x;
    thrust::device_vector<double> position_y;
	thrust::device_vector<double> position_next_x;
    thrust::device_vector<double> position_next_y;
    thrust::device_vector<double> velocity_x;
    thrust::device_vector<double> velocity_y; 

    thrust::device_vector<int> g_status;
	thrust::device_vector<char> g_bm;
	double limit;
	
	void birth_robot();       // pushes back one more robot data to the device_vectors
    void advance_robot();    // launches the move that adds velocity to positions
	bool g_if_collision(unsigned int, unsigned int, bool, bool, double, double);
#endif

	/* population size */
	unsigned int pop_size;

	/* entities in population */
	vector<individual> entities;

	/* Number of objectives */
	unsigned int num_objs;

	/* Dimension of simulation */
	unsigned int dim;

	double dim_limit;


	/* objectives for population, represented by drawables */
	vector<objective *> objectives;

	/* bitmap of all entities, 1 means collison detected */
	vector<one_bit> bm; 

	/* Grid for detecting collisions */
	vector<unsigned int> **grid;
	unsigned int cell_size;
	unsigned int grid_size;

	/* construct functions */
	population() = delete;
	population(unsigned int size, unsigned int dimension, double radius, double limit, unsigned int num_objectives, double objective_radius, unsigned int mode);

	/* AI Perception function, each entity takes in input from the environment */
	void sense(double sense_dist);
	void sense_objectives(double sense_dist);
	void sense_entities(double sense_dist);

	/* AI Decision function, each entity makes a decision based on their current state */
	void decide(double sense_dist);
	void decide_link_objective(double sense_dist);
	void decide_path(double sense_dist);
	void decide_link_entity(double sense_dist);

	/* collision detection among all entities in this population */
	bool collision();

	/* collision detection only use after initialization */
	bool init_collision();

	/* adjust velocity of each entity with respect to collision detection */
	void adjustment();

	/* check if all entities terminate */
	bool terminate();

	void form_path(individual *linked1, individual *linked2, individual *finder);

	void init_grid(double radius, double dimension_limit);
	void clear_grid();
	void assign_to_grid();
	
	/* TODO: for genetic algorithm to calculate fitness */
	void calc_fitness() {}
};

/* TODO: class to fit genetic algorithm */
class demo
{
public:
	/* generation count */
	unsigned int generation_cnt;

	/* population */
	population pop;

	/* construct functions */
	demo();

	/* genetic algorithm stuff */
	void selection();
	void crossover();
	void mutation();
	void add_offspring();
};
