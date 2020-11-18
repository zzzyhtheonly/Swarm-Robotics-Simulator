#include <GL/freeglut.h>
#include <GL/gl.h>
#include <iostream>
#include <unistd.h>

#include "headers.h"

/* arguments */
unsigned int mode = 0;
unsigned int population_size = 10;
double radius = 20;
double ground_dimension = 1000;
unsigned int number_objectives = 2;
double objective_radius = 100;
unsigned int branch_len = 5;
double sense_dist = 100;

/* those are fixed at the moment */
unsigned int dimension_size = 2;

void clear_screen()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
}

void render_function()
{
	clear_screen();
	glOrtho(-ground_dimension, ground_dimension, -ground_dimension, ground_dimension, -ground_dimension, ground_dimension);
	
	/* draw something */
	unsigned long timestamp = 1;	
	population test = population(population_size, dimension_size, radius, ground_dimension, number_objectives, objective_radius, mode);

	while(timestamp++){
		/* Draw objectives, no AI here */
		glColor3f(1.0, 0.0, 0.0);
		for(unsigned int i = 0; i < test.num_objs; ++i){
			test.objectives[i]->draw();
		}

		/* AI Loop: Perception -> Decision -> Action */
		/* Preception */

		test.sense(sense_dist);		

		/* Decision */

		test.decide(sense_dist);

		/* Action */
		/* TODO: GPU version */
		for(unsigned int i = 0; i < test.pop_size; ++i){
			if (test.entities[i].status == LINK) {
				if (test.entities[i].link->branch)
					glColor3f(0.0, 1.0, 0.5);
				else
					glColor3f(0.0, 1.0, 0.0);
			} else if (test.entities[i].status == PATH) {
				glColor3f(.5, 0.0, .5);
			} else {
				glColor3f(0.0, 0.0, 1.0);
			}
			test.entities[i].draw();
			test.entities[i].move_prediction();
		}

		test.adjustment();
		
		/* TODO: GPU version */
		for(unsigned int i = 0; i < test.pop_size; ++i){
			test.entities[i].move();
		}

		/* for leftmost mode, check if all entities reach the rightmost */
		if(mode == LEFTMOST_INIT && test.terminate()){
			break;
		}

		/* Draw lines to better visualize what nodes are doing */
		for(unsigned int i = 0; i < test.pop_size; ++i){
			if (test.entities[i].status == LINK || test.entities[i].status == PATH) {
				if (test.entities[i].status == LINK)
					glColor3f(0.0, 0.5, 0.0);
				else
					glColor3f(0.25, 0.0, 0.25);
				glBegin(GL_LINES);
					vector<double> B_pos = test.entities[i].link->previous->node->pos;
					glVertex2f(test.entities[i].pos[0], test.entities[i].pos[1]);
					glVertex2f(B_pos[0], B_pos[1]);
				glEnd();
			}
		}	


		glFlush();
		clear_screen();
	}

	glFlush();
}

int main(int argc, char* argv[])
{
	/* parse arguments */
	int opt;
	if(argc == 1){
		std::cout << "Usage: ./simulator -mode [population size] "
			  << "[radius of individual: default is set to 20] "
			  << "[dimension of the playground: default is set to 1000]" 
			  << "[number of objectives: default is 2] "
			  << "[radius of objectives: default is 100] "
			  << "[minimum length before branching: default is 5] "
			  << "[Sensing distance: default is 100 "
			  << std::endl;
		std::cout << "Example: ./simulator 10" << std::endl;
		std::cout << "         ./simulator 50 50 2000 " << std::endl;
		std::cout << "         ./simulator 100 20 1000 2 100" << std::endl;
		std::cout << "Note: exit the program by entering Ctrl^C from the terminal" << std::endl;
		std::cout << "Mode: -r randomly initialized without targets" << std::endl;
		std::cout << "      -t robots start from the leftmost edge moving to the rightmost edge" << std::endl;
		return 0;
	}

	while((opt = getopt(argc, argv, "rt")) != -1)
	{
		switch(opt)
		{
			case 'r':
				mode = RANDOM_INIT;
				break;
			case 't':
				mode = LEFTMOST_INIT;
				break;
			case '?':
				std::cout << "Unknown option: " << optopt << std::endl;
				return 0;
				break;
		}
	}

	if(optind < argc){
		population_size = atoi(argv[optind++]);
	}
	if(optind < argc){
		radius = atof(argv[optind++]);
	}
	if(optind < argc){
		ground_dimension = atof(argv[optind++]);
	}
	if (optind < argc){
		number_objectives = atoi(argv[optind++]);
	}
	if (optind < argc){
		objective_radius = atof(argv[optind++]);
	}

	/* init window */
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Robots simulator demo");
	/* display */
	glutDisplayFunc(render_function);
	glutMainLoop();
	return 0;
}
