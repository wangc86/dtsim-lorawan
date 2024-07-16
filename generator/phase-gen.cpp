// Author: Chao Wang (cw@ntnu.edu.tw)
// Description: Read a network topology and generate a random
//              list of initial time offset (aka phase) for
//              each sensor node, AND re-place the sensors
//              that cannot connect to static gateways.

#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<cmath>
#include <stdbool.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <assert.h>

enum simulation_type {ideal, w_prediction, wo_prediction};

#define DEBUG_LOG std::cout << "(" << clock_ticks << ")  "   

int SF7_COMM_RANGE;
int VERTEX_NUM;
int MAP_WIDTH;
int MAP_LENGTH;
int total_links = 0;
int big_delta = 0;
int PERIOD = 60;

struct vertex
{
	int pid;
	int x;
	int y;
    int degree = 0;
    int phase;
    int sf = 12;
    int which_gw = 0;
};

// SF7, 30; SF8, 60; SF9, 90; SF10, 120; SF11, 150; SF12, 180
int sf_range[13] = {0,0,0,0,0,0,0,30,60,90,120,150,180};
float sf_energy[13] = {0,0,0,0,0,0,0,26.33,46.27,85.49,149.25,339.99,592.55};

int num_outliers = 0;

bool within_range (vertex node, int px, int py, int comm_range)
{
    int x = node.x - px;
    int y = node.y - py;
    return ((x*x + y*y) <= (comm_range*comm_range));
}

void sf_assignment (vertex node[])
{
    // assumption on the locations of static gateways
    int gwx[25], gwy[25];
    for (int i = 0; i < 25; i++)
    {
        gwx[i] = 125 + 250*(i%5);
        gwy[i] = 125 + 250*(i/5);
    }
    // by this we implicitly ensure that SF12 always work for at least one static gateway

    // assign SF value to each end device in a greedy fashion
    int sf;

    for (int i = 0; i < VERTEX_NUM; i++)
    {
        for (sf = 7; sf <= 12; sf++)
        {
            bool done = false;
            for (int j = i; j < i + 25; j++)
            {
                if (within_range (node[i], gwx[j%25], gwy[j%25], sf_range[sf])) {
                    node[i].sf = sf;
                    node[i].which_gw = j%25+3;// 0: unused; 1: mobile gateway 1; 2: mobile gateway 2
                    done = true;
                    break;
                }
            }
            if (done) break;
        }
        if (sf > 12)
        {
            std::cerr << "outlier: (" << node[i].x << "," << node[i].y << ")" << std::endl;;
            exit(1);
        }
    }
}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: ./a.out [topology file]" << std::endl;
        return 1;
    }

	srand(time(NULL));

	int i, j, k;	// loop indicator
	int distance, x_offset, y_offset;
	
    std::ifstream ifs (argv[1], std::ifstream::in);
    char discard[50], value[6];

    // The followings are hard-coded to consume the identifiers, etc. in the file
    ifs >> discard; ifs >> discard; ifs >> value;
    SF7_COMM_RANGE = atoi (value);
    ifs >> discard; ifs >> discard; ifs >> value;
    VERTEX_NUM = atoi (value);
    ifs >> discard; ifs >> discard; ifs >> value;
    MAP_WIDTH = atoi (value);
    ifs >> discard; ifs >> discard; ifs >> value;
    MAP_LENGTH = atoi (value);
    ifs >> discard; ifs >> discard; ifs >> discard; ifs >> value;
    //total_links = atoi (value);
    ifs >> discard; ifs >> discard; ifs >> discard; ifs >> value;
    //big_delta = atoi (value);
/*
    std::cout << "SF7_COMM_RANGE = " << SF7_COMM_RANGE << std::endl;
    std::cout << "VERTEX_NUM = " << VERTEX_NUM << std::endl;
    std::cout << "MAP_WIDTH = " << MAP_WIDTH << std::endl;
    std::cout << "MAP_LENGTH = " << MAP_LENGTH << std::endl;
	std::cout << "Total links = " << total_links << std::endl;
	std::cout << "Capital delta = " << big_delta << std::endl;
*/

	struct vertex node[VERTEX_NUM];

    // initialize the data structure for each node
	for(i = 0; i < VERTEX_NUM; i++)
	{
		node[i].pid = i;
        ifs >> value;
        node[i].x = atoi (value);
        ifs >> value;
        node[i].y = atoi (value);
        // initialize the phase of periodic upload
        node[i].phase = rand() % PERIOD;
        //std::cout << node[i].phase << std::endl;
	}

    ifs.close();
/*
    for (i = 0; i < VERTEX_NUM; i++)
    {
        std::cout << node[i].x << ' ' << node[i].y << std::endl;
    }
*/

	// Reconstruct neighbor relations
	for (i = 0; i < VERTEX_NUM; i++)
	{
		for (j = i + 1; j < VERTEX_NUM; j++)
//		for(j = 0; j < VERTEX_NUM; j++) // <---- redundant
		{		

			x_offset = node[j].x - node[i].x; // always non-negative, thanks to qsort.
			if (x_offset > SF7_COMM_RANGE)
			{
				break; // <-- this is correct optimization, but may be unnecessary 
			}

			y_offset = node[i].y - node[j].y; // could be negative, but nevermind.
			distance = x_offset*x_offset + y_offset*y_offset;
			if (distance <= SF7_COMM_RANGE*SF7_COMM_RANGE)
			{
				total_links += 1;
                node[i].degree += 1;
                node[j].degree += 1;
			}

            if (big_delta < node[j].degree)
            {
                big_delta = node[j].degree;
            }
		}

        if (big_delta < node[i].degree)
        {
            big_delta = node[i].degree;
        }
	}

//  Verify the results
//	std::cout << "Total links = " << total_links << std::endl;
//	std::cout << "Capital delta = " << big_delta << std::endl;

    // Assign initial SF values
    sf_assignment (node);

//    std::ofstream ofs ("p.txt", std::ofstream::out);

    std::cout << "SF7_COMM_RANGE = " << SF7_COMM_RANGE << std::endl;
    std::cout << "VERTEX_NUM = " << VERTEX_NUM << std::endl;
    std::cout << "MAP_WIDTH = " << MAP_WIDTH << std::endl;
    std::cout << "MAP_LENGTH = " << MAP_LENGTH << std::endl;
	std::cout << "Total links = " << total_links << std::endl;
	std::cout << "Capital delta = " << big_delta << std::endl;

    int sf1112 = 0;
	for (i = 0; i < VERTEX_NUM; i++)
	{
        if (node[i].sf >= 11)
        {
            sf1112 ++;
        }
    }

    std::cout << "SF1112 % = " << ((float) sf1112)/((float)VERTEX_NUM)*100 << std::endl;
    std::cout << "# of outliers = " << num_outliers << std::endl;
    std::cout << "DATA PERIOD = " << PERIOD << std::endl;
    std::cout << "------------------------------" << std::endl;
    std::cout << "x  y  SF  phase which_gateway: " << std::endl;

	for (i = 0; i < VERTEX_NUM; i++)
	{
        std::cout << node[i].x << " " << node[i].y << " " << node[i].sf << " " << node[i].phase << " " << node[i].which_gw << std::endl;
    }

    //ofs.close();

	return 0;
}
