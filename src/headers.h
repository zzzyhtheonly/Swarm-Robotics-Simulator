#include <vector>

using namespace::std;

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
};

/* class represents a single robot entity */
class individual
{
public:
	/* dimension of our space, fixed to 2 at this moment */
	unsigned int dimension;

	/* coordinate limitation */
	double limit;

	/* current status */
	states status;

	/* position of each entity, size should be fixed to 2 at this moment, we dont use pair for efficiency here because it may be extended to 3 dimension one day */
	vector<double> pos;
	
	/* next position of each entity with respect to the current velocity, it is used to adjust velocity in order to avoid collision */
	vector<double> pos_next;

	/* velocity for each direction, should coincide with dimension of pos */
	vector<double> velocity;

	/* radius for entity since it is a circle */
	double radius;

	/* construct functions */
	individual() = delete;
	individual(unsigned int dimension, double radius, double limit);

	/* no checks move */
	void _move(vector<double>&);

	/* movement with respect to velocity, affects pos */
	void move();

	/* movement prediction, affects pos_next */
	void move_prediction();

	/* draw circle */
	void draw();

	/* collision detection between this and given entity */
	bool if_collision(individual another);

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

	/* bitmap of all entities, 1 means collison detected */
	vector<one_bit> bm; 

	/* construct functions */
	population() = delete;
	population(unsigned int size, unsigned int dimension, double radius, double limit);

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
