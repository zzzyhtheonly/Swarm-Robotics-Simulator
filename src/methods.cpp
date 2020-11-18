#include <GL/gl.h>
#include <GL/freeglut.h>
#include <math.h>
#include <bits/stdc++.h>

#include "headers.h"

linked_tree::linked_tree(objective *r, linked_tree *p, drawable *n) {
	root = r;
	previous = p;
	node = n;
	if (p == NULL) {
		this->branch = true;
		this->branch_dist = 0;
	} else {
		/* Was the previous node an objective? */
		if ((drawable *)root == previous->node)
			branch_dist = 1;
		/* Otherwise it has to be another entity */
		else
			branch_dist = previous->branch_dist+1;
		/* This link is a branch if it meets global variable branch_len */
		if (branch_dist >= branch_len) {
			branch = true;
			branch_dist = 0;
		}
		/* Unless the previous entity was a branch it is no longer able to link with another entity */ 
		if (!previous->branch)
			previous->free = false;
	}
}

/** Parent class for things that need to be displayed on screen **/
drawable::drawable(unsigned int dimension, double radius, double limit)
{
	this->dimension = dimension;
	this->limit = limit;
	this->pos = vector<double>(dimension, 0);
	this->radius = radius;

	/* initialize coordinates randomly */
	/* TODO: make it into float */
	random_device dev;
	mt19937 rng(dev());
	uniform_int_distribution<mt19937::result_type> dist(0, limit);
	uniform_int_distribution<mt19937::result_type> sign(0, 1);

	for(unsigned int i = 0; i < dimension; ++i) {
		double val = dist(rng);
		this->pos[i] = sign(rng) ? val : -val;
	}
}

/* draw itself in OpenGL by filling with tiny triangles */
void drawable::draw()
{
	unsigned int count = 20;
	GLfloat twicePi = 2.0f * M_PI;

	glBegin(GL_TRIANGLE_FAN);

		/* center */
		glVertex2f(pos[0], pos[1]);

		for(unsigned int i = 0; i <= count; ++i) {
			glVertex2f(pos[0] + (radius * cos(i * twicePi / count)), pos[1] + (radius * sin(i * twicePi / count)));
		}
	glEnd();
}

objective::objective(unsigned int dimension, double radius, double limit) : drawable(dimension, radius, limit) {
	/* Only thing objective needs to do is establish root of linked tree */
	this->link = new linked_tree(this, NULL, this);
};

/* 
 * dimension: fix to 2 at the moment
 * radius: fix to 20 at the moment
 * limit: playground dimension limit, only square allowed, assume that you want a 1000*1000 square then limit should be 1000
 */
individual::individual(unsigned int dimension, double radius, double limit) : drawable(dimension, radius, limit)
{
	this->dimension = dimension;
	this->limit = limit;
	this->pos = vector<double>(dimension, 0);
	this->pos_next = vector<double>(dimension, 0);
	this->velocity = vector<double>(dimension, 0);
	this->radius = radius;
	this->status = READY;

	/* initialize coordinates and velocities randomly */
	/* TODO: make it into float */
	random_device dev;
	mt19937 rng(dev());
	uniform_int_distribution<mt19937::result_type> dist(0, limit);
	uniform_int_distribution<mt19937::result_type> sign(0, 1);

	double fixed_velocity = ((double)limit / 10000.0);
	for(unsigned int i = 0; i < dimension; ++i) {
		double val = dist(rng);
		this->pos[i] = this->pos_next[i] = sign(rng) ? val : -val;
		this->velocity[i] = sign(rng) ? fixed_velocity : -fixed_velocity;
	}
}

/* pure move */
void individual::_move(vector<double>& pos)
{
	for(unsigned int i = 0; i < this->dimension; ++i){
		/* detect collision with walls */
		double tmp = pos[i] + this->velocity[i];	
		if(tmp > limit || tmp < -limit){
			velocity[i] = -velocity[i];
		}
		pos[i] += velocity[i];
	}
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
	_move(this->pos_next);
}

/*
 * test base on pos_next because we make sure that there is no collision at initial
 * another: a specific entity to test collision
 */
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

bool individual::if_collision(drawable another)
{
	double distance = 0;

	for(unsigned int i = 0; i < this->dimension; ++i){
		double tmp = this->pos_next[i] - another.pos[i];
		distance += (tmp * tmp);
	}

	distance = sqrt(distance);

	return distance < this->radius + another.radius ? true : false;
}

bool individual::if_sense(individual another, double sense_dist) {
	double distance = 0;

	for(unsigned int i = 0; i < this->dimension; ++i){
		double tmp = this->pos_next[i] - another.pos[i];
		distance += (tmp * tmp);
	}

	distance = sqrt(distance);

	return distance < sense_dist + another.radius ? true : false;	
}

void population::sense(double sense_dist)
{
	
	for (unsigned int i = 0; i < this->pop_size; ++i){
		if (this->entities[i].status == LINK) continue;
		/* Is an entity on top of an objective? */
		for (unsigned int j = 0; j < this->num_objs; ++j) {
			if (this->entities[i].if_collision(this->objectives[j])) {
				std::cout << "Entity " << i << " is on objective " << j << std::endl;
				this->entities[i].status = ON_OBJ;
			}
		}
		/* Continue to next entity if on objective */
		if (this->entities[i].status == ON_OBJ)
			continue;
		/* Is an entity within sensing distance of another entity
		who is in a linked tree? */
		for (unsigned int j = 0; j < this->pop_size; ++j) {
			if (i == j) continue;
			if (this->entities[i].if_sense(this->entities[j], sense_dist) && 
				this->entities[j].status == LINK) {
				std::cout << "Entity " << i << " sensed entity " << j << std::endl;
				this->entities[i].status = SENSE;
			}
		}
	}
}

void population::decide(double sense_dist)
{
	for (unsigned int i = 0; i < this->pop_size; ++i){
		/* If the entity sensed that it was on an objective */
		if (this->entities[i].status == ON_OBJ) {
			/* Find objective that this entity is on top of */
			objective *obj_tmp;
			unsigned int j;
			for (j = 0; j < this->num_objs; ++j) {
				if (this->entities[i].if_collision(this->objectives[j])) {
					obj_tmp = &(this->objectives[j]);
					break;
				}
			}
			/* not sure if this can happen, but return to running if so */
			if (obj_tmp == NULL) {
				this->entities[i].status = RUNNING;
				continue;
			}
			/* Otherwise this entity is now in a linked_tree */
			std::cout << "Entity " << i << " is linking with objective " << j << std::endl;
			this->entities[i].status = LINK;
			this->entities[i].velocity = vector<double>(this->dim, 0);
			this->entities[i].link = new linked_tree(obj_tmp->link->root, obj_tmp->link, &(this->entities[i]));
		/* If the entity sensed that it was within sensing distance of at least one
		other linked entity */
		} else if (this->entities[i].status == SENSE) {
			/* Find the first linked entity that is also free or able to branch */
			std::cout << "Entity " << i << " is deciding what to do after sensing another linked entity" << std::endl;
			individual *another_tmp;
			unsigned int k;
			for (k = 0; k < this->pop_size; ++k) {
				if (i == k) continue;
				if (this->entities[i].if_sense(this->entities[k], sense_dist) &&
					this->entities[k].status == LINK) {
					if (this->entities[k].link->free || this->entities[k].link->branch) {
						std::cout << "Entity " << i << " is going to link with entity " << k << std::endl;
						another_tmp = &(this->entities[k]);
						break;
					}
				}
			}
			/* This can happen if two entities sense the same linked entity */
			if (k >= this->pop_size) {
				std::cout << "Entity " << i << " could not find another suitable entity to link with" << std::endl;
				this->entities[i].status = RUNNING;
				continue;
			}
			/* Entity is within sensing distance of a linked entity, so create another link */
			std::cout << "Starting entity " << i << " linking process" << std::endl;
			another_tmp->link->free = false;
			std::cout << "Other entity link free set to false" << std::endl;
			this->entities[i].status = LINK;
			std::cout << "Entity " << i << " status set to LINK" << std::endl;
			this->entities[i].velocity = vector<double>(this->dim, 0);
			std::cout << "Entity " << i << " velocity set to 0" << std::endl;
			this->entities[i].link = new linked_tree(another_tmp->link->root, another_tmp->link, &(this->entities[i]));
			std::cout << "Entity " << i << " linked successfully" << std::endl;
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

/*
 * size: fix to 10 at the moment
 * others: same to the arguments of individual(...)
 */
population::population(unsigned int size, unsigned int dimension, double radius, double limit, unsigned int num_objectives, double objective_radius)
{
	this->pop_size = size;
	this->num_objs = num_objectives;
	this->dim = dimension;

	for(unsigned int i = 0; i < size; ++i){
		this->entities.push_back(individual(dimension, radius, limit));
		this->bm.push_back(one_bit());
	}

	for(unsigned int i = 0; i < num_objectives; ++i) {
		this->objectives.push_back(objective(dimension, objective_radius, limit));
	}

	unsigned int retries = 0;
	/* make sure the population is initialized with no collision, give a retry limitation to prevent forever loop */
	while(collision() && retries++ < 99){
		/* TODO: GPU version */
		for(unsigned int i = 0; i < size; ++i){
			if(this->bm[i].bit){
				this->bm[i].bit = 0;
				this->entities[i] = individual(dimension, radius, limit);
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
