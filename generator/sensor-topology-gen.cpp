// Author: Chao Wang (cw@ntnu.edu.tw)
// Description: Generate a network topology
//              that simulates the deployment
//              of LoRaWAN end devices.
//              The initial time offset for each sensor node is zero.

#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<math.h>
#include <stdbool.h>
#include <iostream>
#include <fstream>

struct vertex
{
	int pid;
	int x;
	int y;
    int degree;
};

int cmpfunc (const void * a, const void * b)
{
	return ( (*(struct vertex*)a).x - (*(struct vertex*)b).x );
}

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cerr << "Usage: ./a.out [VERTEX_NUM]" << std::endl;
        return 1;
    }

	srand(time(NULL));

	int i, j, k;	// loop indicator
	int distance, x_offset, y_offset;
	
	int total_links = 0;
    int big_delta = 0;

//	printf("%d\n", VERTEX_NUM);

	// initialization
	int SF7_COMM_RANGE = 30; // SF7, 30; SF8, 60; SF9, 90; SF10, 120; SF11, 150; SF12, 180
	//int VERTEX_NUM = 20;
	int VERTEX_NUM = atoi (argv[1]);
    int MAP_WIDTH = 1250;
    int MAP_LENGTH = 1250;
//	scanf("%d %d\n", &COMM_RANGE, &VERTEX_NUM);
	struct vertex node[VERTEX_NUM];
	for(i = 0; i < VERTEX_NUM; i++)
	{
		node[i].pid = i;
		//scanf("%d %d\n", &node[i].x, &node[i].y);
        node[i].x = (rand() % MAP_WIDTH) + 1;
        node[i].y = (rand() % MAP_LENGTH) + 1;
        node[i].degree = 0;
	}
	
	qsort(node, VERTEX_NUM, sizeof(struct vertex), cmpfunc);
	
	// compute neighbor relations
	for (i = 0; i < VERTEX_NUM; i++)
	{
		for (j = i + 1; j < VERTEX_NUM; j++)
//		for(j = 0; j < VERTEX_NUM; j++) // <---- redundant
		{		

			x_offset = node[j].x - node[i].x; // always non-negative, thanks to qsort.
			if (x_offset > SF7_COMM_RANGE)
			{
				break;
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

    std::cout << "SF7_COMM_RANGE = " << SF7_COMM_RANGE << std::endl;
    std::cout << "VERTEX_NUM = " << VERTEX_NUM << std::endl;
    std::cout << "MAP_WIDTH = " << MAP_WIDTH << std::endl;
    std::cout << "MAP_LENGTH = " << MAP_LENGTH << std::endl;
	std::cout << "Total links = " << total_links << std::endl;
	std::cout << "Capital delta = " << big_delta << std::endl;
	std::cerr << "Total links = " << total_links << std::endl;
	std::cerr << "Capital delta = " << big_delta << std::endl;

    for (i = 0; i < VERTEX_NUM; i++)
    {
        std::cout << node[i].x << ' ' << node[i].y << std::endl;
    }

	return 0;
}
