#include <GL/gl.h>
#include <GL/freeglut.h>
#include <math.h>
#include <bits/stdc++.h>

#include "headers.h"

/** Parent class for things that need to be displayed on screen **/
drawable::drawable(unsigned int dimension, double radius, double limit)
{
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

objective::objective(unsigned int dimension, double radius, double limit) : drawable(dimension, radius, limit) {};

/* 
 * dimension: fix to 2 at the moment
 * radius: fix to 20 at the moment
 * limit: playground dimension limit, only square allowed, assume that you want a 1000*1000 square then limit should be 1000
 */
individual::individual(unsigned int dimension, double radius, double limit, unsigned int mode) : drawable(dimension, radius, limit)
{
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

void population::sense()
{
	/* Is an entity on top of an objective? If so, stop.
	   TODO: Expand this later to perform more complex behavior */
	for (unsigned int i = 0; i < this->pop_size; ++i){
		for (unsigned int j = 0; j < this->num_objs; ++j) {
			if (this->entities[i].if_collision(this->objectives[j])){
				this->entities[i].status = ON_OBJ;
			}
		}
	}
}

void population::decide()
{
	/* If over an objective, stop.
	   TODO: Expand this later to perform more complex behavior */
	for (unsigned int i = 0; i < this->pop_size; ++i){
		if (this->entities[i].status == ON_OBJ) {
			this->entities[i].status = STOP;
			this->entities[i].velocity = vector<double>(this->dim, 0);
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

/*
 * size: fix to 10 at the moment
 * others: same to the arguments of individual(...)
 */
population::population(unsigned int size, unsigned int dimension, double radius, double limit, unsigned int num_objectives, double objective_radius, unsigned int mode)
{
	this->pop_size = size;
	this->num_objs = num_objectives;
	this->dim = dimension;

	for(unsigned int i = 0; i < size; ++i){
		this->entities.push_back(individual(dimension, radius, limit, mode));
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
				this->entities[i] = individual(dimension, radius, limit, mode);
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
