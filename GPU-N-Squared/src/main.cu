#include <GL/freeglut.h>
#include <GL/gl.h>
#include <iostream>
#include <unistd.h>

#include <time.h>
#include <math.h> 

#include "headers.h"

const string GRD_DIM_STR = "Ground_Dim:";
const string NUM_ENT_STR = "Num_Entities:";
const string RAD_STR = "Radius:";
const string NUM_OBJ_STR = "Num_Objectives:";
const string OBJ_RAD_STR = "Objective_Radius:";
const string CLR_STR = "CLEAR";

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

ofstream log_file("log.txt");
bool log_intermediate = false;

#ifdef GPU
__global__
void device_print_something(unsigned int pop_size, double* pos_x, double* pos_y, char* bm)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= pop_size) return;
	
	printf("%f %f\n", pos_x[i], pos_y[i]);
}
#endif

ifstream log_in("log2.txt");

void clear_screen()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
}

vector<string> tokenize(string line) {
        vector<string> words;
        int i = 0;
        int j = 0;
        while (i < line.size()) {
                if (line[i] == '\t' || i == line.size()-1) {
                        words.push_back(line.substr(j, i-j+(i==line.size()-1)));
                        j = i+1;
                }
                i++;
        }
        return words;
}

void parse_global_params(vector<string> words) {
	if (words[0] == GRD_DIM_STR) {
		ground_dimension = stod(words[1]);
		std::cout << GRD_DIM_STR << " " << ground_dimension << std::endl;
	} else if (words[0] == NUM_ENT_STR) {
		population_size = stoi(words[1]);
		std::cout << NUM_ENT_STR << " " << population_size << std::endl;
	} else if (words[0] == RAD_STR) {
		radius = stod(words[1]);
		std::cout << RAD_STR << " " << radius << std::endl;
	} else if (words[0] == NUM_OBJ_STR) {
		number_objectives = stoi(words[1]);
		std::cout << NUM_OBJ_STR << " " << number_objectives << std::endl;
	} else if (words[0] == OBJ_RAD_STR) {
		objective_radius = stod(words[1]);
		std::cout << OBJ_RAD_STR << " " << objective_radius << std::endl;
	}
}

void draw_circle(bool isObj, double x, double y){
	unsigned int count = 20;
        GLfloat twicePi = 2.0f * M_PI;
	double r = radius;
	if (isObj) r = objective_radius;

	glBegin(GL_TRIANGLE_FAN);
		glVertex2f(x, y);

                for(unsigned int i = 0; i <= count; ++i) {
                        glVertex2f(x + (r * cos(i * twicePi / count)), y + (r * sin(i * twicePi / count)));
                }
	glEnd();
}

void render_log_function() {
	clear_screen();
        glOrtho(-ground_dimension, ground_dimension, -ground_dimension, ground_dimension, -ground_dimension, ground_dimension);

	string line;
	vector<string> words;
	double x,y,r,g,b;
	int id;

        while(getline(log_in, line)) {
                words = tokenize(line);
                if (words.size() <= 1) {
                        glFlush();
                        clear_screen();
                } else {
                        id = stoi(words[0]);
                        x = stod(words[1]);
                        y = stod(words[2]);
                        r = stod(words[3]);
                        g = stod(words[4]);
                        b = stod(words[5]);
                        glColor3f(r,g,b);
                        draw_circle((id>=population_size), x,y);
                }
        }
        return;
}

void render_log(int *argcp, char **argv)
{
	string line;
	for (int i = 0; i < 5; i++) {
		getline(log_in, line);
		parse_global_params(tokenize(line));
	}

	/* init window */
        glutInit(argcp, argv);
        glutInitDisplayMode(GLUT_SINGLE);
        glutInitWindowSize(500, 500);
        glutInitWindowPosition(100, 100);
        glutCreateWindow("Robots simulator demo - rendering log file");
        /* display */
        glutDisplayFunc(render_log_function);
        glutMainLoop();

	return;
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
	//cout << "end of gpu version" << endl;
	
#if 0
	dim3 blocksPerGrid(ceil((population_size)/16.0), 1, 1);
	dim3 threadsPerBlock(16, 1, 1);
	double* d_position_x =  thrust::raw_pointer_cast(&test.position_x[0]);
	double* d_position_y =  thrust::raw_pointer_cast(&test.position_y[0]);
	char* d_bm =  thrust::raw_pointer_cast(&test.g_bm[0]);
	device_print_something<<<blocksPerGrid,threadsPerBlock>>>(population_size, d_position_x, d_position_y, d_bm);
#endif
	log_file << "Initialization time: " << init_time << std::endl;
	//return;
#endif
  
	double r,g,b;
	while(timestamp++){
		/* Draw objectives, no AI here */
		glColor3f(1.0, 0.0, 0.0);
		r = 1.; g = 0.; b = 0.;
		check = clock();
		for(unsigned int i = 0; i < test.num_objs; ++i){
			test.objectives[i]->draw(r,g,b);
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
				if (test.entities[i].link->branch) {
					glColor3f(0.0, 1.0, 0.5);
					r = 0.; g = 1.; b = .5;
				} else {
					glColor3f(0.0, 1.0, 0.0);
					r = 0.; g = 1.; b = 0.;
				}
			} else if (test.entities[i].status == PATH) {
				glColor3f(.5, 0.0, .5);
				r = .5; g = 0.; b = .5;
			} else {
				glColor3f(0.0, 0.0, 1.0);
				r = 0.; g = 0.; b = 1.;
			}

			test.entities[i].draw(r,g,b);
		}
		draw_entities_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		check = clock();
#ifdef GPU
		test.predict_robot();
#else
		for(unsigned int i = 0; i < test.pop_size; ++i){
			test.entities[i].move_prediction();
		}
#endif
		move_prediction_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		check = clock();
		test.adjustment();
		adjustment_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;
		
		/* TODO: GPU version */
		check = clock();
#ifdef GPU
		test.advance_robot();
#else
        for(unsigned int i = 0; i < test.pop_size; ++i){
                test.entities[i].move();
		}
#endif
		move_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;

		/* for leftmost mode, check if all entities reach the rightmost */
		if(mode == LEFTMOST_INIT && test.terminate()){
			break;
		}

		check = clock();
#ifndef GPU
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
#endif
		draw_lines_time += ((double)(clock() - check))/ CLOCKS_PER_SEC;	

		if (log_intermediate)
			test.draw();

		double total_time = ((double)(clock() - start))/ CLOCKS_PER_SEC;
		if (total_time >= max_time) {
			double unaccounted = total_time;
			unaccounted -= init_time + draw_obj_time + draw_entities_time + sense_time;
			unaccounted -= decide_time + move_prediction_time + adjustment_time;
			unaccounted -= move_time + draw_lines_time;
			
			log_file << "\nSimulation Complete\n--------------------" << std::endl;
			log_file << "Ran for " << total_time << " seconds." << std::endl;
			log_file << "Number of updates: " << timestamp-1 << std::endl;
			log_file << "\nTiming breakdown\n--------------------" << std::endl;
			log_file << "\tInitialization: " << (init_time/total_time)*100. << "%%" << std::endl;
			log_file << "\tDrawing objectives: " << (draw_obj_time/total_time)*100. << "%%" << std::endl;
			log_file << "\tDrawing entities: " << (draw_entities_time/total_time)*100.<<"%%"<<std::endl;
			log_file << "\tSensing: " << (sense_time/total_time)*100. << "%%" << std::endl;
			log_file << "\tDeciding: " << (decide_time/total_time)*100. << "%%" << std::endl;
			log_file << "\tPredicting moves: "<<(move_prediction_time/total_time)*100.<<"%%"<<std::endl;
			log_file << "\tAdjustments: " << (adjustment_time/total_time)*100. << "%%" <<std::endl;
			log_file << "\tMoving: " << (move_time/total_time)*100. << "%%" << std::endl;
			log_file << "\tDrawing lines: " << (draw_lines_time/total_time)*100. << std::endl;
			log_file << "\tTime unaccounted for: " << (unaccounted/total_time)*100. << std::endl;
			log_file << "\nNumber of updates: " << timestamp-1 << std::endl;

			test.draw();
#ifdef GPU
			log_file.close();
#endif
			exit(0);
		}

		glFlush();
		clear_screen();
		if (log_intermediate)
			std::cout << CLR_STR << std::endl;
	}

	glFlush();
}

int main(int argc, char* argv[])
{
	cudaSetDevice(1);
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

	while((opt = getopt(argc, argv, "lirt")) != -1)
	{
		switch(opt)
		{
			case 'l':
				render_log(&argc, argv);
				exit(0);
			case 'i':
				log_intermediate = true;
				break;
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

	std::cout << GRD_DIM_STR << "\t" << std::to_string(ground_dimension) << std::endl
		<< NUM_ENT_STR << "\t" << std::to_string(population_size) << std::endl
		<< RAD_STR << "\t" << std::to_string(radius) << std::endl
		<< NUM_OBJ_STR << "\t" << std::to_string(number_objectives) << std::endl
		<< OBJ_RAD_STR << "\t" << std::to_string(radius) << std::endl;

#ifdef GPU
	render_function();
#else
	/* init window */
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Robots simulator demo");
	/* display */
	glutDisplayFunc(render_function);
	glutMainLoop();
#endif
	return 0;
}
