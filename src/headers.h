#include <vector>

using namespace::std;

class drawable;
class objective;
class individual;

/* define a one bit data structure for bitmap */
typedef struct one_bit
{
	unsigned bit:1;

	one_bit() { bit = 0; }
}one_bit;

/* states describe a robot's status */
enum states
{
	STOP,
	READY,
	RUNNING,
	ON_OBJ,
	LINK,
	SENSE,
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
	/* What is at this link in the linked tree */
	drawable *node;
	/* Distance from last branch */
	unsigned int branch_dist;
	/* Indicates if the tree can branch at this link */
	bool branch = false;
	/* Indicates if another entity can link off this link */
	bool free = true;

	linked_tree() = delete;
	linked_tree(objective *r, linked_tree *p, drawable *n);
};

class drawable
{
public:
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
	drawable(unsigned int dimension, double radius, double limit);

	/* draw circle */
	void draw();
};

/* Used to differentiate objectives from other drawables
   Can expand this later */
class objective : public drawable {
public:
	/* construct functions */
	objective() = delete;
	objective(unsigned int dimension, double radius, double limit);

};

/* class represents a single robot entity */
class individual : public drawable
{
public:
	/* current status */
	states status;
	
	/* next position of each entity with respect to the current velocity, it is used to adjust velocity in order to avoid collision */
	vector<double> pos_next;

	/* velocity for each direction, should coincide with dimension of pos */
	vector<double> velocity;

	/* construct functions */
	individual() = delete;
	individual(unsigned int dimension, double radius, double limit);

	/* no checks move */
	void _move(vector<double>&);

	/* movement with respect to velocity, affects pos */
	void move();

	/* movement prediction, affects pos_next */
	void move_prediction();

	/* collision detection between this and given entity */
	bool if_collision(individual another);
	bool if_collision(drawable another);

	/* Sensing detection between this and given entity */
	bool if_sense(individual another, double sense_dist);

	/* TODO: for genetic algorithm to calculate fitness */
	void calc_fitness() {}
};

/* class represents  */
class population
{
public:
	/* population size */
	unsigned int pop_size;

	/* entities in population */
	vector<individual> entities;

	/* Number of objectives */
	unsigned int num_objs;

	/* Dimension of simulation */
	unsigned int dim;

	/* objectives for population, represented by drawables */
	vector<objective> objectives;

	/* bitmap of all entities, 1 means collison detected */
	vector<one_bit> bm; 

	/* construct functions */
	population() = delete;
	population(unsigned int size, unsigned int dimension, double radius, double limit, unsigned int num_objectives, double objective_radius);

	/* AI Perception function, each entity takes in input from the environment */
	void sense(double sense_dist);

	/* AI Decision function, each entity makes a decision based on their current state */
	void decide(double sense_dist);

	/* collision detection among all entities in this population */
	bool collision();

	/* adjust velocity of each entity with respect to collision detection */
	void adjustment();
	
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
