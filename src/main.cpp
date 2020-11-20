#include <GL/freeglut.h>
#include <GL/gl.h>
#include <iostream>
#include <fstream>

#include "headers.h"

/* arguments */
unsigned int population_size = 10;
double radius = 20;
double ground_dimension = 1000;

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
	glColor3f(0.0, 0.0, 1.0);
	glOrtho(-ground_dimension, ground_dimension, -ground_dimension, ground_dimension, -ground_dimension, ground_dimension);
	
	/* draw something */
	unsigned long timestamp = 1;	
	population test = population(population_size, dimension_size, radius, ground_dimension);

	while(timestamp++){
		/* TODO: iteration */
		if (timestamp == 20000) break;
		/* TODO: GPU version */
		for(unsigned int i = 0; i < test.pop_size; ++i){
			test.entities[i].draw();
			test.entities[i].move_prediction();
		}

		test.adjustment();
		
		/* TODO: GPU version */
		for(unsigned int i = 0; i < test.pop_size; ++i){
			test.entities[i].move();
		}

		glFlush();
		clear_screen();
	}

	ofstream out("out.txt");
        if (out.is_open()) {
            for(unsigned int i = 0; i < test.pop_size; ++i){
				out << "(" << test.entities[i].pos[0] << ","
				<< test.entities[i].pos[1] << ")";
		}
        out.close();
    }

	glFlush();
}

int main(int argc, char* argv[])
{
	/* parse arguments */
	if(argc == 1){
		std::cout << "Usage: ./simulator [population size] "
			  << "[radius of individual: default is set to 20] "
			  << "[dimension of the playground: default is set to 1000]" 
			  << std::endl;
		std::cout << "Example: ./simulator 10" << std::endl;
		std::cout << "         ./simulator 50 50 2000 " << std::endl;
		std::cout << "Note: exit the program by entering Ctrl^C from the terminal" << std::endl;
		return 0;
	}
	if(argc >= 2){
		population_size = atoi(argv[1]);
	}
	if(argc >= 3){
		radius = atof(argv[2]);
	}
	if(argc >= 4){
		ground_dimension = atof(argv[3]);
	}
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Robots simulator demo");
	/* display */
	glutDisplayFunc(render_function);
	glutMainLoop();

	render_function();

	return 0;
}
