#include <bits/stdc++.h>
#include <algorithm>
#include "headers.h"

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
	this->dimension = dimension;
	this->limit = limit;
	this->pos = vector<double>(dimension, 0);
	this->radius = radius;
	this->id = id;

	/* initialize coordinates randomly */
	random_device dev;
	mt19937 rng(dev());
	uniform_real_distribution<double> dist(0, limit);
	uniform_int_distribution<mt19937::result_type> sign(0, 1);

	for(unsigned int i = 0; i < dimension; ++i) {
		double val = dist(rng);
		this->pos[i] = sign(rng) ? val : -val;
	}
}

/* draw itself in OpenGL by filling with tiny triangles */
void drawable::draw(double r, double g, double b)
{
	log_file << this->id << "\t" << this->pos[0] << "\t" << this->pos[1]
		<< "\t" << r << "\t" << g << "\t" << b << std::endl;
	/*
	unsigned int count = 20;
	GLfloat twicePi = 2.0f * M_PI;

	glBegin(GL_TRIANGLE_FAN);

		// center
		glVertex2f(pos[0], pos[1]);

		for(unsigned int i = 0; i <= count; ++i) {
			glVertex2f(pos[0] + (radius * cos(i * twicePi / count)), pos[1] + (radius * sin(i * twicePi / count)));
		}
	glEnd(); */
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
	//cout << distance << endl;

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

/* test if collision exists otherwise update collision bitmap 
	comparisons are only made to nearby entities */
bool population::collision()
{

	bool res = false;
	unsigned int left_x, right_x, up_y, down_y;
	double l = this->dim_limit;
	double c = this->cell_size;
	double r = this->entities[0].radius*2;

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

	return res;
}

/* customized collsion test only used after initialization */
bool population::init_collision()
{
	bool res = false;
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
	std::cout << "FOUND A PATH" << std::endl;
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
	std::cout << "Grid is " << grid_size << " by " << grid_size << " with cell size " << cell_size << std::endl;
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

	for(unsigned int i = 0; i < size; ++i){
		this->entities.push_back(individual(dimension, radius, limit, mode, i));
		this->bm.push_back(one_bit());
	}

	for(unsigned int i = 0; i < num_objectives; ++i) {
		objective *tmp = new objective(dimension, objective_radius, limit, i);
		this->objectives.push_back(tmp);
		this->bm.push_back(one_bit());
	}

	unsigned int retries = 0;
	/* make sure the population is initialized with no collision, give a retry limitation to prevent forever loop */
	while(init_collision() && retries++ < 99){
		/* TODO: GPU version */
		for(unsigned int i = 0; i < size; ++i){
			if(this->bm[i].bit){
				this->bm[i].bit = 0;
				this->entities[i] = individual(dimension, radius, limit, mode, i);
			}
		}

		for(unsigned int i = 0; i < num_objectives; ++i){
			if(this->bm[size+i].bit){
				this->bm[size+i].bit = 0;
				/* FIXME: memory leak here, we need a destructor to make it works properly */
				this->objectives[i] = new objective(dimension, objective_radius, limit, i);
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
