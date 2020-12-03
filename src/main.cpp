#include <GL/freeglut.h>
#include <GL/gl.h>
#include <iostream>
#include <unistd.h>
#include <time.h> 

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
unsigned int max_time = 300;

/* those are fixed at the moment */
unsigned int dimension_size = 2;

// Source: http://www.david-amador.com/2012/09/how-to-take-screenshot-in-opengl/
bool save_screenshot(string filename, int w, int h)
{	
  //This prevents the images getting padded 
 // when the width multiplied by 3 is not a multiple of 4
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
 
  int nSize = w*h*3;
  // First let's create our buffer, 3 channels per Pixel
  char* dataBuffer = (char*)malloc(nSize*sizeof(char));
 
  if (!dataBuffer) return false;
 
   // Let's fetch them from the backbuffer	
   // We request the pixels in GL_BGR format, thanks to Berzeger for the tip
   glReadPixels((GLint)0, (GLint)0,
		(GLint)w, (GLint)h,
		 GL_BGR, GL_UNSIGNED_BYTE, dataBuffer);
 
   //Now the file creation
   FILE *filePtr = fopen(filename.c_str(), "wb");
   if (!filePtr) return false;
 
 
   unsigned char TGAheader[12]={0,0,2,0,0,0,0,0,0,0,0,0};
   unsigned char header[6] = { w%256,w/256,
			       h%256,h/256,
			       24,0};
   // We write the headers
   fwrite(TGAheader,	sizeof(unsigned char),	12,	filePtr);
   fwrite(header,	sizeof(unsigned char),	6,	filePtr);
   // And finally our image data
   fwrite(dataBuffer,	sizeof(GLubyte),	nSize,	filePtr);
   fclose(filePtr);
 
   free(dataBuffer);
 
  return true;
}

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

	/* timing accumulators */
	clock_t check;
	double init_time = 0;
	double draw_obj_time = 0;
	double draw_entities_time = 0;
	double sense_time = 0;
	double decide_time = 0;
	double move_prediction_time = 0;
	double adjustment_time = 0;
	double move_time = 0;
	double draw_lines_time = 0;
	clock_t start = clock();

	check = clock();
	population test = population(population_size, dimension_size, radius, ground_dimension, number_objectives, objective_radius, mode);
	init_time = ((double)(clock() - check))/ CLOCKS_PER_SEC;

#ifdef GPU
	cout << "end of gpu version" << endl;
	return;
#endif

	while(timestamp++){
		/* Draw objectives, no AI here */
		glColor3f(1.0, 0.0, 0.0);
		
		check = clock();
		for(unsigned int i = 0; i < test.num_objs; ++i){
			test.objectives[i]->draw();
		}
		draw_obj_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		/* AI Loop: Perception -> Decision -> Action */
		/* Preception */

		check = clock();
		test.sense(sense_dist);		
		sense_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		/* Decision */

		check = clock();
		test.decide(sense_dist);
		decide_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		/* Action */
		/* TODO: GPU version */
		check = clock();
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
		}
		draw_entities_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		check = clock();
		for(unsigned int i = 0; i < test.pop_size; ++i){
			test.entities[i].move_prediction();
		}
		move_prediction_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		check = clock();
		test.adjustment();
		adjustment_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;
		
		/* TODO: GPU version */
		check = clock();
		for(unsigned int i = 0; i < test.pop_size; ++i){
			test.entities[i].move();
		}
		move_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		/* for leftmost mode, check if all entities reach the rightmost */
		if(mode == LEFTMOST_INIT && test.terminate()){
			break;
		}

		check = clock();
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
		draw_lines_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;	

		double total_time = ((double)(clock() - start))/ CLOCKS_PER_SEC;
		if (total_time >= max_time) {
			double unaccounted = total_time;
			unaccounted -= init_time + draw_obj_time + draw_entities_time + sense_time;
			unaccounted -= decide_time + move_prediction_time + adjustment_time;
			unaccounted -= move_time + draw_lines_time;

			std::cout << "Ran for " << total_time << " seconds." << std::endl;
			std::cout << "Timing breakdown\n--------------------" << std::endl;
			printf("\tInitialization: %.2f%%\n", (init_time/total_time)*100.);
			printf("\tDrawing objectives: %.2f%%\n", (draw_obj_time/total_time)*100.);
			printf("\tDrawing entities: %.2f%%\n", (draw_entities_time/total_time)*100.);
			printf("\tSensing: %.2f%%\n", (sense_time/total_time)*100.);
			printf("\tDeciding: %.2f%%\n", (decide_time/total_time)*100.);
			printf("\tPredicting moves: %.2f%%\n", (move_prediction_time/total_time)*100.);
			printf("\tAdjustments: %.2f%%\n", (adjustment_time/total_time)*100.);
			printf("\tMoving: %.2f%%\n", (move_time/total_time)*100.);
			printf("\tDrawing lines: %.2f%%\n", (draw_lines_time/total_time)*100.);
			printf("\tTime unaccounted for: %.2f%%\n", (unaccounted/total_time)*100.);
			save_screenshot("out.tga", 500, 500);
			exit(0);
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
			  << "[max time: in seconds, default is 300] "
			  << std::endl;
		std::cout << "Example: ./simulator 10" << std::endl;
		std::cout << "         ./simulator 50 50 2000 " << std::endl;
		std::cout << "         ./simulator 100 20 1000 2 100 300" << std::endl;
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
	if (optind < argc){
		max_time = atoi(argv[optind++]);
	}

#if 0
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
#endif
	render_function();
	return 0;
}
