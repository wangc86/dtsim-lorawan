//TODO: remove the counting of possible catches since they are wrong; also, adjust the parsing scripts accordingly.
// Author: Chao Wang (cw@ntnu.edu.tw)
// Description: A discrete-time simulator
//              Read a network topology and simulate a scenario of
//              a mobile gateway cruising along a patterned path,
//              to estimate the percentage of uploads that could be
//              made via the mobile gateway.
//              And to record the upload matrix at each time slot.
//              The upload matrix U(t):
/*                         staticGateway1 staticGateway2 mobileGateway1 mobileGateway2
                   sensor1     0                1             0               0
                   sensor2     0                0             0               0
                   sensor3     0                0             1               0
                     ...

                An entry (i,j) is zero if no successful upload at the time slot t,
                and one if sensor i made a successful upload via gateway j at t.
*/
// Latest update on 2024/7/16

#include<stdio.h>
#include<stdlib.h>
#include<time.h>
#include<cmath>
#include <stdbool.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <assert.h>
#include <queue>

//#define SETTING_TYPE 1
//#define SETTING_TYPE 2
#define SETTING_TYPE 4

enum algorithm_type {ideal, w_prediction, wo_prediction};
//#define EVALUATION_TYPE ideal
#define EVALUATION_TYPE w_prediction
//#define EVALUATION_TYPE wo_prediction

//int speed = 1;
int speed = 6; // 6 m/s = 21.6 km/h

// # of faults at any moment
#define FN 0

// whether or not there's a jammer
#define JM 1

enum gateway_type {na, m_gw1, m_gw2}; // and 3, 4, 5, ..., 27 are static gateways

int TOTAL_TICKS = 3600;

#define DEBUG_LOG std::cout << "(" << clock_ticks << ")  "   

int SF7_COMM_RANGE;
int VERTEX_NUM;
int MAP_WIDTH;
int MAP_LENGTH;
int big_delta = 0;

int clock_ticks = 0;
int PERIOD; // set by the input file instead
//int PERIOD = 1000; // for testing the benefit of long periods
int T_REJOIN = 5; // time for an end device to rejoin the LoRaWAN and transmit data

int D; // data freshness requirement; set to 3*PERIOD

struct vertex
{
	int pid;
	int x;
	int y;
    int degree = 0;
    int phase;
    int sf = 12;
    std::queue<int> buffered_data;
    std::queue<int> baseline_buffered_data;
    int upload_count = 0;
    int catches = 0;
    int possible_catches = 0;

    // The followings are for arrival prediction
    char* arrival_observed_history;
    char* arrival_real_history;
    int nzeros_count = 0;
    int learned_slack = 0;
    int expected_slack = 0;
    bool active_learning = true;
    int tries = 0;

    // for CPS fault tolerance
    bool faulty = false;
    int consecutive_faults = 0;

    // for timeliness measurement
    std::queue<int> latencies;
    std::queue<int> latencies_baseline;

    // for upload matrix
    int which_gw; // from config file
    bool upload_made_just_now;
    int upload_via; // actual gateway at runtime
};

struct area
{
	int x;
	int y;
};

area** faulty_comm; // cases: 1, 3, 5, 7, or 9 CPS faults at any moment
int total_faults = 0;
int sf1112_faults = 0;
int fault_range = 10;
//int fault_range = 25;
//int fault_range = 500;

area jammer;
int jamming_range = 50;


// SF7, 30; SF8, 60; SF9, 90; SF10, 120; SF11, 150; SF12, 180
int sf_range[13] = {0,0,0,0,0,0,0,30,60,90,120,150,180};
float sf_energy[13] = {0,0,0,0,0,0,0,26.33,46.27,85.49,149.25,339.99,592.55};

int num_outliers = 0;

long double total_energy_baseline_sf710 = 0;
long double total_energy_baseline_sf1112 = 0;
long double total_energy_sf1112 = 0;
long double total_catches = 0;

void print_and_clear_upload_matrix (vertex node[])
{
    int i, j;
	for (i = 0; i < VERTEX_NUM; i++)
	{
/*
        if (node[i].upload_made_just_now) {
            for (j = 1; j <= 27; j++)
            {
                if (j == node[i].upload_via) std::cerr << "1 ";
                else std::cerr << "0 ";
            } 
        } else {
            for (j = 1; j <= 27; j++)
            {
                std::cerr << "0 ";
            } 
        }
        std::cerr << std::endl;
        node[i].upload_made_just_now = false;
        node[i].upload_via = node[i].which_gw;
*/
        if (node[i].upload_made_just_now) {
            std::cerr << i+1 << " " << node[i].upload_via << " ";
            node[i].upload_made_just_now = false;
            node[i].upload_via = node[i].which_gw;
        }
    }
    std::cerr << std::endl;
}

bool within_range (vertex node, int px, int py, int comm_range)
{
    int x = node.x - px;
    if (abs(x) > comm_range) return false;
    int y = node.y - py;
    if (abs(y) > comm_range) return false;
    return ((x*x + y*y) <= (comm_range*comm_range));
}

void learn_arrival_pattern (vertex& n, bool arrived) // This is for mobile gateway arrival prediction
{
    n.tries ++;

    if (arrived) // Note: arrived is true as long as 'any' of the mobile gateways arrived.
    {
        strcat (n.arrival_observed_history, "1");
        // store the learning result
        n.learned_slack = n.nzeros_count;
        n.expected_slack = n.learned_slack;
        // start the next learning phase
        n.nzeros_count = 0;
        n.active_learning = false;
        n.tries = 0;
    }
    else
    {
        strcat (n.arrival_observed_history, "0");
        n.nzeros_count ++;
        if (n.tries > 30) // so that we do not learn forever; so in the random-walk case, prediction strategy case may be reduced to the no prediction case, in the long run.
        {
        //// store the learning result //FIXME: verify these two lines
        //n.learned_slack = n.nzeros_count;
        //n.expected_slack = n.learned_slack;
            n.active_learning = false;
            n.nzeros_count = 0;
            n.tries = 0;
        }
    }
}

bool caught_mobile_gateway (vertex& n, int mx, int my, int mx2, int my2, int range)
{
    if (within_range (n, mx, my, SF7_COMM_RANGE)) {
        n.catches ++;
        total_catches ++;
        total_energy_sf1112 += sf_energy[7];
        n.latencies.push (clock_ticks - n.buffered_data.front() + 1);
        n.buffered_data.pop ();
        n.upload_via = m_gw1;
        n.upload_made_just_now = true;
        return true;
    } else if (within_range (n, mx2, my2, SF7_COMM_RANGE)) {
        n.catches ++;
        total_catches ++;
        total_energy_sf1112 += sf_energy[7];
        n.latencies.push (clock_ticks - n.buffered_data.front() + 1);
        n.buffered_data.pop ();
        n.upload_via = m_gw2;
        n.upload_made_just_now = true;
        return true;
    } else {
        return false;
    }
}

void try_mobile_gateway (vertex& n, int mx, int my, int mx2, int my2)
{
    bool didnt_turn_off = false;
    bool didnt_turn_on = false;

    if (n.active_learning)
    {
        learn_arrival_pattern (n,
                               (within_range(n, mx, my, SF7_COMM_RANGE)
                               || within_range(n, mx2, my2, SF7_COMM_RANGE)));
        total_energy_sf1112 += 180; // the energy cost for one-second receive window
        if (caught_mobile_gateway (n, mx, my, mx2, my2, SF7_COMM_RANGE))
        {
            if (!n.buffered_data.empty()) // chance to piggyback one additional data in payload :)
            {
                n.catches ++;
                total_catches ++;
                n.latencies.push (clock_ticks - n.buffered_data.front() + 1);
                n.buffered_data.pop ();
            }
        }
    }
    else // i.e., predicting using what we've learned
    {
        if (n.expected_slack == 0)
        {
            if (!caught_mobile_gateway (n, mx, my, mx2, my2, SF7_COMM_RANGE))
            {
                didnt_turn_off = true;
                total_energy_sf1112 += sf_energy[7]; // we need to add this, for an attempt was made
                // TODO: maybe do this only after multiple failures?
                n.active_learning = true;
            }
            else if (!n.buffered_data.empty()) // chance to piggyback one additional data in payload :)
            {
                n.catches ++;
                total_catches ++;
                n.latencies.push (clock_ticks - n.buffered_data.front() + 1);
                n.buffered_data.pop ();
            }
            n.expected_slack = n.learned_slack;
        }
        else
        {
            if (within_range (n, mx, my, SF7_COMM_RANGE)) // ground true
            {
                didnt_turn_on = true;
            }
            n.expected_slack --;
        }
    }

    // build real history
    if (within_range (n, mx, my, SF7_COMM_RANGE)
        || within_range (n, mx2, my2, SF7_COMM_RANGE))
    {
        n.possible_catches ++;
        if (didnt_turn_on)
        {
            strcat (n.arrival_real_history, "_");
        }
        else
        {
            strcat (n.arrival_real_history, "1");
        }
    }
    else
    {
        if (didnt_turn_off)
        {
            strcat (n.arrival_real_history, ".");
        }
        else
        {
            strcat (n.arrival_real_history, "0");
        }
    }
}

int search_for_usable_sf (vertex n, vertex node[])
{
    int sf = 11;
    /* Now our system does not use helper node
    for (int i = 0; i < VERTEX_NUM; i++)
    {
        if (i == n.pid || node[i].faulty) continue;
        for (int j = 7; j <= sf; j++) // SF11 and SF12 we do not use here since they are power hungry
        {
            if (within_range (n, node[i].x, node[i].y, sf_range[j]))
            {
                if (j < sf)
                {
                    sf = j;
                    break;
                }
            }
        }
    }
    */
    return sf;
}

void baseline_operation (vertex node[], int i)
{
    if (!node[i].faulty) // i.e., either as normal or return to normal
    {
        node[i].upload_count ++;
        if (node[i].sf < 11)
        {
            node[i].latencies.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
            node[i].upload_made_just_now = true;
            node[i].latencies_baseline.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
        }
        else
        {
            node[i].latencies_baseline.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
        }
        node[i].baseline_buffered_data.pop ();
        node[i].consecutive_faults = 0;
/*        if (!node[i].baseline_buffered_data.empty()) // batching (?)
        {
            if (node[i].sf < 11)
            {
                total_energy_baseline_sf710 += sf_energy[node[i].sf];
                node[i].latencies.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
                node[i].latencies_baseline.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
            }
            else
            {
                total_energy_baseline_sf1112 += sf_energy[node[i].sf];
                node[i].latencies_baseline.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
            }
            node[i].upload_count ++;
            node[i].baseline_buffered_data.pop ();
        }
*/
    }
    else
    {
        total_faults ++;
        if (node[i].consecutive_faults >= 3)
        {
            int sf = search_for_usable_sf (node[i], node);
            if (sf <= 10)
            {
                if (node[i].consecutive_faults > 0)
                {
                    if (node[i].sf < 11)
                    {
                        total_energy_baseline_sf710 += sf_energy[sf];
                        node[i].latencies.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
                        node[i].upload_made_just_now = true;
                        node[i].latencies_baseline.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
                    }
                    else
                    {
                        total_energy_baseline_sf1112 += sf_energy[sf];
                        node[i].latencies_baseline.push (clock_ticks - node[i].baseline_buffered_data.front() + 1);
                    }
                    node[i].upload_count ++;
                    node[i].baseline_buffered_data.pop ();
                    node[i].consecutive_faults --;
                }
            }
        }
    }
}

void update_node (vertex node[], int mx, int my, int mx2, int my2)
{
    for (int i = 0; i < VERTEX_NUM; i++)
    {
        node[i].faulty = false;
        for (int j = 0; j < FN; j++) // this for loop is the timing performance bottleneck
        {
            if (faulty_comm[j][clock_ticks].x - fault_range > node[i].x) break;
            if (within_range (node[i], faulty_comm[j][clock_ticks].x, faulty_comm[j][clock_ticks].y, fault_range))
            {
                node[i].faulty = true;
                node[i].consecutive_faults ++;
                break;
            }
        }
        if (within_range (node[i], jammer.x, jammer.y, jamming_range))
        {
            if (!node[i].faulty) {
                node[i].faulty = true;
                node[i].consecutive_faults ++;
            }
        }

        if (node[i].sf < 11) // i.e., a near-mode node
        {
            if ((clock_ticks - node[i].phase) % PERIOD == 0) // the default data-sending moment
            {
                total_energy_baseline_sf710 += sf_energy[node[i].sf];
                node[i].baseline_buffered_data.push (clock_ticks);
                baseline_operation (node, i);
            }

            if ((!node[i].baseline_buffered_data.empty())
                &&((clock_ticks - node[i].baseline_buffered_data.front()) > PERIOD)) 
 //               &&((clock_ticks - node[i].baseline_buffered_data.front()) % PERIOD == 0))
            {
                total_energy_baseline_sf710 += sf_energy[node[i].sf];
                baseline_operation (node, i);
            }

        }
        else // i.e., a far-mode node
        {
            // baseline type
            if ((clock_ticks - node[i].phase) % PERIOD == 0) // the default data-sending moment
            {
                total_energy_baseline_sf1112 += sf_energy[node[i].sf];
                node[i].baseline_buffered_data.push (clock_ticks);

                node[i].buffered_data.push (clock_ticks); // this is for the following evaluation types.
                if (node[i].faulty == true)
                {
                    sf1112_faults ++;
                }

                baseline_operation (node, i);
            }

            if ((!node[i].baseline_buffered_data.empty())
                &&((clock_ticks - node[i].baseline_buffered_data.front()) > PERIOD))
//                &&((clock_ticks - node[i].baseline_buffered_data.front()) % PERIOD == 0))
            {
                total_energy_baseline_sf1112 += sf_energy[node[i].sf];
                baseline_operation (node, i);
            }

            switch (EVALUATION_TYPE)
            {
            case ideal: // send using SF7 whenever possible
                if (!node[i].buffered_data.empty())
                {
                    if (caught_mobile_gateway (node[i], mx, my, mx2, my2, SF7_COMM_RANGE))
                    {
                        // note that many updates were performed in the function caught_mobile_gateway()
                        node[i].possible_catches ++;
                    }
                    else if (((clock_ticks - (node[i].buffered_data.front())) >= PERIOD*2-1)) // fall back
/*                        ||
                        (((clock_ticks - node[i].buffered_data.front()) > PERIOD*2)
                        &&((clock_ticks - node[i].buffered_data.front()) % PERIOD == 0)))
*/                    {
                        total_energy_sf1112 += sf_energy[node[i].sf];
                        if (node[i].faulty)
                        {
                            sf1112_faults ++;
                            if (node[i].consecutive_faults >= 3)
                            {
                                int sf = search_for_usable_sf (node[i], node);
                                if (sf <= 10)
                                {
                                    total_energy_sf1112 += sf_energy[sf]; // FIXME: need to lookahead future faults
                                }
                                else
                                {
                                    continue;
                                }
                            }
                        }
                        node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                        node[i].upload_made_just_now = true;
                        node[i].buffered_data.pop ();
                        node[i].consecutive_faults = 0;

                        if (!node[i].buffered_data.empty()) // chance to piggyback one additional data in payload :)
                        {
                            node[i].catches ++;
                            total_catches ++;
                            node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                            node[i].buffered_data.pop ();
                        }
                    }
                }
                break;
            case w_prediction:
                if (node[i].sf == 11) // one receive window
                {
                    if ((clock_ticks - node[i].phase) % PERIOD == 0) 
                    {
                        // first try 
                        try_mobile_gateway (node[i], mx, my, mx2, my2);
                    }
                    else if ((!node[i].buffered_data.empty())
//                        && (((clock_ticks - (node[i].buffered_data.front())) % PERIOD) == 0)) // fall back
                        && ((clock_ticks - (node[i].buffered_data.front())) > PERIOD)) // fall back
//                        &&((clock_ticks - node[i].buffered_data.front()) > PERIOD)
//                        &&((clock_ticks - node[i].buffered_data.front()) % PERIOD == 0))
                    {
                        total_energy_sf1112 += sf_energy[node[i].sf];
                        if (node[i].faulty)
                        {
                            sf1112_faults ++;
                            if (node[i].consecutive_faults >= 3)
                            {
                                int sf = search_for_usable_sf (node[i], node);
                                if (sf <= 10)
                                {
                                    total_energy_sf1112 += sf_energy[sf]; // FIXME: need to lookahead future faults
                                }
                                else
                                {
                                    continue;
                                }
                            }
                        }
                        node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + T_REJOIN);
                        node[i].upload_made_just_now = true;
                        node[i].buffered_data.pop ();
                        node[i].consecutive_faults = 0;

                        if (!node[i].buffered_data.empty()) // chance to piggyback one additional data in payload :)
                        {
                            node[i].catches ++;
                            total_catches ++;
                            node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                            node[i].buffered_data.pop ();
                        }
                    }
                }
                else if (node[i].sf == 12) // two receive windows
                {
                    if ((clock_ticks - node[i].phase) % PERIOD == 0) 
                    {
                        // first try 
                        try_mobile_gateway (node[i], mx, my, mx2, my2);
                    }
                    else if ((!node[i].buffered_data.empty())
                        && ((clock_ticks - (node[i].buffered_data.front())) == D/2)) // watch out if 2 does not divide D 
                    {
                        // second try 
                        try_mobile_gateway (node[i], mx, my, mx2, my2);
                    }
                    else if ((!node[i].buffered_data.empty())
//                        && (((clock_ticks - (node[i].buffered_data.front())) % PERIOD) == 0)) // fall back
                        && ((clock_ticks - (node[i].buffered_data.front())) > D/2)) // fall back
//                        &&((clock_ticks - node[i].buffered_data.front()) > D/2)
//                        &&((clock_ticks - node[i].buffered_data.front()) % PERIOD == 0))
                    {
                        total_energy_sf1112 += sf_energy[node[i].sf];
                        if (node[i].faulty)
                        {
                            sf1112_faults ++;
                            if (node[i].consecutive_faults >= 3)
                            {
                                int sf = search_for_usable_sf (node[i], node);
                                if (sf <= 10)
                                {
                                    total_energy_sf1112 += sf_energy[sf]; // FIXME: need to lookahead future faults
                                }
                                else
                                {
                                    continue;
                                }
                            }
                        }
                        node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + T_REJOIN);
                        node[i].upload_made_just_now = true;
                        node[i].buffered_data.pop ();
                        node[i].consecutive_faults = 0;

                        if (!node[i].buffered_data.empty()) // chance to piggyback one additional data in payload :)
                        {
                            node[i].catches ++;
                            total_catches ++;
                            node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                            node[i].buffered_data.pop ();
                        }
                    }
                }
                break;
            case wo_prediction:
                if (node[i].sf == 11) // one receive window
                {
                    if ((clock_ticks - node[i].phase) % PERIOD == 0) 
                    {
                        // try once using an one-second receive window
                        total_energy_sf1112 += 180; // the energy cost for one-second receive window
                        if (caught_mobile_gateway (node[i], mx, my, mx2, my2, SF7_COMM_RANGE))
                        {
                            if (!node[i].buffered_data.empty()) // chance to piggyback one additional data in payload :)
                            {
                                node[i].catches ++;
                                total_catches ++;
                                node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                                node[i].buffered_data.pop ();
                            }
                        }
                    }
                    else if ((!node[i].buffered_data.empty()) 
//                        && (((clock_ticks - (node[i].buffered_data.front())) % PERIOD) == 0)) // fall back
                        && ((clock_ticks - (node[i].buffered_data.front())) > PERIOD)) // fall back
//                        &&((clock_ticks - node[i].buffered_data.front()) > PERIOD)
//                        &&((clock_ticks - node[i].buffered_data.front()) % PERIOD == 0))
                    {
                        total_energy_sf1112 += sf_energy[node[i].sf];
                        if (node[i].faulty)
                        {
                            sf1112_faults ++;
                            if (node[i].consecutive_faults >= 3)
                            {
                                int sf = search_for_usable_sf (node[i], node);
                                if (sf <= 10)
                                {
                                    total_energy_sf1112 += sf_energy[sf]; // FIXME: need to lookahead future faults
                                }
                                else
                                {
                                    continue;
                                }
                            }
                        }
                        node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + T_REJOIN);
                        node[i].upload_made_just_now = true;
                        node[i].buffered_data.pop ();
                        node[i].consecutive_faults = 0;

                        if (!node[i].buffered_data.empty()) // chance to piggyback one additional data in payload :)
                        {
                            node[i].catches ++;
                            total_catches ++;
                            node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                            node[i].buffered_data.pop ();
                        }
                    }
                }
                else if (node[i].sf == 12) // two receive windows
                {
                    if ((clock_ticks - node[i].phase) % PERIOD == 0) 
                    {
                        // try once using an one-second receive window
                        total_energy_sf1112 += 180; // the energy cost for one-second receive window
                        if (caught_mobile_gateway (node[i], mx, my, mx2, my2, SF7_COMM_RANGE))
                        {
                            if (!node[i].buffered_data.empty()) // chance to piggyback one additional data in payload :)
                            {
                                node[i].catches ++;
                                total_catches ++;
                                node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                                node[i].buffered_data.pop ();
                            }
                        }
                    }
                    else if ((!node[i].buffered_data.empty())
                        && ((clock_ticks - (node[i].buffered_data.front())) == D/2)) // watch out if 2 does not divide D 
                    {
                        // try again using an one-second receive window
                        total_energy_sf1112 += 180; // the energy cost for one-second receive window
                        if (caught_mobile_gateway (node[i], mx, my, mx2, my2, SF7_COMM_RANGE))
                        {
                            if (!node[i].buffered_data.empty()) // chance to piggyback one additional data in payload :)
                            {
                                node[i].catches ++;
                                total_catches ++;
                                node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                                node[i].buffered_data.pop ();
                            }
                        }
                    }
                    else if ((!node[i].buffered_data.empty())
//                        && (((clock_ticks - (node[i].buffered_data.front())) % PERIOD) == 0)) // fall back
                        && ((clock_ticks - (node[i].buffered_data.front())) > D/2)) // fall back
//                        &&((clock_ticks - node[i].buffered_data.front()) > D/2)
//                        &&((clock_ticks - node[i].buffered_data.front()) % PERIOD == 0))
                    {
                        total_energy_sf1112 += sf_energy[node[i].sf];
                        if (node[i].faulty)
                        {
                            sf1112_faults ++;
                            if (node[i].consecutive_faults >= 3)
                            {
                                int sf = search_for_usable_sf (node[i], node);
                                if (sf <= 10)
                                {
                                    total_energy_sf1112 += sf_energy[sf]; // FIXME: need to lookahead future faults
                                }
                                else
                                {
                                    continue;
                                }
                            }
                        }
                        node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + T_REJOIN);
                        node[i].upload_made_just_now = true;
                        node[i].buffered_data.pop ();
                        node[i].consecutive_faults = 0;

                        if (!node[i].buffered_data.empty()) // chance to piggyback one additional data in payload :)
                        {
                            node[i].catches ++;
                            total_catches ++;
                            node[i].latencies.push (clock_ticks - node[i].buffered_data.front() + 1);
                            node[i].buffered_data.pop ();
                        }
                    }
                }
                break;
            }
        }
    }
}

/*
void update_mobile (int mx, int my, vertex node[], int* mdelta)
{
    int new_degree = 0;
//std::cout << "mx " << mx << " my " << my << std::endl;

	for (int i = 0; i < VERTEX_NUM; i++)
	{
		if (within_range (node[i], mx, my, SF7_COMM_RANGE))
		{
            new_degree ++;
		}
	}

    if (*mdelta < new_degree)
    {
        *mdelta = new_degree;
    }
}
*/
void move_regular(int &mx, int &my, int &mx2, int &my2)
{
    // TODO
}

void move_random(int &mx, int &my, int &mx2, int &my2)
{
	srand(time(NULL));
    switch (rand() % 4){
    case 0:
        my += speed;
        if (my >= MAP_LENGTH)
            my = MAP_LENGTH - 1;
        break;
    case 1:
        mx += speed;
        if (mx >= MAP_WIDTH)
            mx = MAP_WIDTH - 1;
        break;
    case 2:
        my -= speed;
        if (my <= 0)
            my = 1;
        break;
    case 3:
        mx -= speed;
        if (mx <= 0)
            mx = 1;
        break;
    }
    switch (rand() % 4){
    case 0:
        my2 += speed;
        if (my2 >= MAP_LENGTH)
            my2 = MAP_LENGTH - 1;
        break;
    case 1:
        mx2 += speed;
        if (mx2 >= MAP_WIDTH)
            mx2 = MAP_WIDTH - 1;
        break;
    case 2:
        my2 -= speed;
        if (my2 <= 0)
            my2 = 1;
        break;
    case 3:
        mx2 -= speed;
        if (mx2 <= 0)
            mx2 = 1;
        break;
    }
}

void move_jammer() // random walk
{
    int s = 1; // moving speed
	srand(time(NULL));
    switch (rand() % 4){
    case 0:
        jammer.y += s;
        if (jammer.y >= MAP_LENGTH)
            jammer.y = MAP_LENGTH - 1;
        break;
    case 1:
        jammer.x += s;
        if (jammer.x >= MAP_WIDTH)
            jammer.x = MAP_WIDTH - 1;
        break;
    case 2:
        jammer.y -= s;
        if (jammer.y <= 0)
            jammer.y = 1;
        break;
    case 3:
        jammer.x -= s;
        if (jammer.x <= 0)
            jammer.x = 1;
        break;
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
    char discard[100], value[6];

/* the following is an example of the header of each input file
SF7_COMM_RANGE = 30
VERTEX_NUM = 18
MAP_WIDTH = 100
MAP_LENGTH = 100
Total links = 17
Capital delta = 3
SF1112 % = 22.2222
# of outliers = 2
DATA PERIOD = 60
------------------------------
x  y  SF  phase which_gateway: 
*/
    // The followings are hard-coded to parse an input file
    ifs >> discard; ifs >> discard; ifs >> value;
    SF7_COMM_RANGE = atoi (value);
    ifs >> discard; ifs >> discard; ifs >> value;
    VERTEX_NUM = atoi (value);
    ifs >> discard; ifs >> discard; ifs >> value;
    MAP_WIDTH = atoi (value);
    ifs >> discard; ifs >> discard; ifs >> value;
    MAP_LENGTH = atoi (value);
    for (int i = 0; i < 20; i++)
    {
        ifs >> discard;
    }

    ifs >> value;
    PERIOD = atoi (value);
    D = 3*PERIOD;

    for (int i = 0; i < 6; i++)
    {
        ifs >> discard;
    }

	struct vertex node[VERTEX_NUM];

    // initialize the data structure for each node
	for(i = 0; i < VERTEX_NUM; i++)
	{
		node[i].pid = i;
        ifs >> value;
        node[i].x = atoi (value);
        ifs >> value;
        node[i].y = atoi (value);
        ifs >> value;
        node[i].sf = atoi (value);
        ifs >> value;
        node[i].phase = atoi (value);
        if (node[i].sf >= 11)
        {
            node[i].arrival_observed_history = (char*) malloc(TOTAL_TICKS);
            node[i].arrival_real_history = (char*) malloc(TOTAL_TICKS);
            memset (node[i].arrival_observed_history, 0, TOTAL_TICKS);
            memset (node[i].arrival_real_history, 0, TOTAL_TICKS);
        }
        ifs >> value;
        node[i].which_gw = atoi (value);
        node[i].upload_made_just_now = false;
        node[i].upload_via = node[i].which_gw;
	}

    ifs.close();
///*
    faulty_comm = new area*[FN];
    for (int i = 0; i < FN; i++)
    {
        faulty_comm[i] = new area[TOTAL_TICKS];
        srand(i+1);
        int x = 0;
        int y = 0;
        int duration = 0;
        int tick = 0;
        while (tick < TOTAL_TICKS)
        {
            x = rand() % MAP_WIDTH;
            y = rand() % MAP_LENGTH;
            duration = 1 + rand() % 240;
            for (int x = 0; x < duration; x++)
            {
                faulty_comm[i][tick].x = x;
                faulty_comm[i][tick].y = y;
                tick++;
                if (tick >= TOTAL_TICKS) break;
            }
        }

        //for (int j = 0; j < 200; j++)
        //{
        //    std::cerr << faulty_comm[i][j].x << " ";
        //    std::cerr << faulty_comm[i][j].y << " ";
        //}
        //std::cerr << std::endl;
    }
    // re-order according to x coordinate to speed up run-time checks
    int t = 0, tmp = 0;
    while (t < TOTAL_TICKS)
    {
        for (int i = 0; i < FN; i++)
        {
            for (int j = i+1; j < FN; j++)
            {
                if (faulty_comm[j][t].x < faulty_comm[i][t].x)
                {
                    tmp = faulty_comm[i][t].x;
                    faulty_comm[i][t].x = faulty_comm[j][t].x;
                    faulty_comm[j][t].x = tmp;
                    tmp = faulty_comm[i][t].y;
                    faulty_comm[i][t].y = faulty_comm[j][t].y;
                    faulty_comm[j][t].y = tmp;
                }
            }
        }
        t++;
    }
//*/

/* // for debug purpose
    for (i = 0; i < VERTEX_NUM; i++)
    {
        std::cout << node[i].x << ' ' << node[i].y << ' ' << node[i].sf << ' ' << node[i].phase << std::endl;
    }
*/

    // Set the initial position of the mobile gateway 1
    int mx = 625;
    int my = 625;
    // Set the initial degree of the mobile gateway 1
    int mdelta = 0;

    // Set the initial position of the mobile gateway 2
    int mx2 = 625;
    int my2 = 625;
    // Set the initial degree of the mobile gateway 2
    int mdelta2 = 0;

#if JM == 0
    jammer.x = 999999;
    jammer.y = 999999;
#else
    jammer.x = 625;
    jammer.y = 1;
#endif


//std::cerr << "---------- start simulation -----------" << std::endl;
    switch (SETTING_TYPE)
    {
    case 2:
        std::cout << "SETTING TYPE: two mobile gateways (patterned)" << std::endl;
        std::cerr << "SETTING TYPE: two mobile gateways (patterned)" << std::endl;
        break;
    case 4:
        std::cout << "SETTING TYPE: two mobile gateways (random walk)" << std::endl;
        std::cerr << "SETTING TYPE: two mobile gateways (random walk)" << std::endl;
        break;
    }

    switch (EVALUATION_TYPE)
    {
    case ideal:
        std::cout << "ALGORITHM TYPE: clairvoyant" << std::endl;
        std::cerr << "ALGORITHM TYPE: clairvoyant" << std::endl;
        break;
    case w_prediction:
        std::cout << "ALGORITHM TYPE: with prediction" << std::endl;
        std::cerr << "ALGORITHM TYPE: with prediction" << std::endl;
        break;
    case wo_prediction:
        std::cout << "ALGORITHM TYPE: without prediction" << std::endl;
        std::cerr << "ALGORITHM TYPE: without prediction" << std::endl;
        break;
    }

    std::cout << "VERTEX_NUM = " << VERTEX_NUM << std::endl;
    std::cerr << "VERTEX_NUM = " << VERTEX_NUM << std::endl;

    //while (clock_ticks < 1000000) 
    while (clock_ticks < TOTAL_TICKS) 
    {
        update_node (node, mx, my, mx2, my2);
        print_and_clear_upload_matrix (node);
        clock_ticks ++;
#if SETTING_TYPE == 2
        move_regular (mx, my, mx2, my2);
#elif SETTING_TYPE == 4
        move_random (mx, my, mx2, my2);
#else
No such configuration. This should not compile.
#endif

#if JM == 0
        ;
#else
        move_jammer ();
#endif
    }



    //std::cout << "---------- Done ------------------------" << std::endl;
/*
    int total_upload = 0;
    int sf1112 = 0;
    int sf1112_uploads = 0;
    int sf1112_possible = 0;
	for (i = 0; i < VERTEX_NUM; i++)
	{
        total_upload += ((clock_ticks - node[i].phase)/PERIOD + 1);
        if (node[i].sf >= 11)
        {
            sf1112 ++;
            sf1112_uploads += ((clock_ticks - node[i].phase)/PERIOD + 1);
            sf1112_possible += node[i].possible_catches;
        }
//        std::cout << node[i].x << " " << node[i].y << " SF = " << node[i].sf << std::endl;
    }
    std::cout << "SF1112_NUM = " << sf1112 << std::endl;
    std::cout << "SF1112 % = " << ((float) sf1112)/((float)VERTEX_NUM)*100 << std::endl;
//    std::cout << "mdelta = " << mdelta << std::endl;
    std::cout << "clock ticks = " << clock_ticks << std::endl;
    std::cout << "A       total uploads = " << total_upload << std::endl;
    std::cout << "B       sf1112 uploads = " << sf1112_uploads << std::endl;
    std::cout << "C       sf1112 possible catches = " << sf1112_possible << std::endl;
    std::cout << "D       sf1112 caughts = " << total_catches << std::endl;
    std::cout << "E       total energy cost (baseline) = " << total_energy_baseline_sf710 + total_energy_baseline_sf1112 << std::endl;
    std::cout << "F       total energy cost = " << total_energy_baseline_sf710 + total_energy_sf1112 << std::endl;
    std::cout << "G       SF1112 energy cost (baseline) = " << total_energy_baseline_sf1112 << std::endl;
    std::cout << "H       SF1112 energy cost = " << total_energy_sf1112 << std::endl;
    std::cout << "C/B     sf1112 catch opportunities = " << ((float)sf1112_possible)/((float)sf1112_uploads)*100 << " %" << std::endl;
    std::cout << "D/A     catch % (near-nodes included) = " << ((float) total_catches)/((float)total_upload)*100 << std::endl;
    std::cout << "D/B     catch % (near-nodes excluded) = " << ((float) total_catches)/((float)sf1112_uploads)*100 << std::endl;
    std::cout << "D/C     catch % (from possible ones) = " << ((float) total_catches)/((float)sf1112_possible)*100 << std::endl;

    std::cout << "(E-F)/E energy saving % = " << (total_energy_baseline_sf1112 - total_energy_sf1112)/(total_energy_baseline_sf710 + total_energy_baseline_sf1112)*100 << std::endl;

    std::cout << "(G-H)/G SF1112 energy saving % = " << (total_energy_baseline_sf1112 - total_energy_sf1112)/(total_energy_baseline_sf1112)*100 << std::endl;

    std::cout << "I       total faults experienced = " << total_faults << std::endl;
    std::cout << "I/A     faults % = " << ((float) total_faults)/((float)total_upload)*100 << std::endl;

    std::cout << "J       sf1112 faults experienced = " << sf1112_faults << " DO NOT USE" << std::endl;
    std::cout << "J/I     sf1112 faults % (among total faults) = " << ((float) sf1112_faults)/((float)total_faults)*100 << " DO NOT USE" << std::endl;

    std::cout << "-----------------------" << std::endl;
    std::cout << "total faults  = " << total_faults << std::endl;
    std::cout << "total faults % = " << ((float) total_faults)/((float)total_upload)*100 << std::endl;
    std::cout << "total energy saved = " << total_energy_baseline_sf710 + total_energy_sf1112 << std::endl;
    std::cout << "total energy saved % = " << (total_energy_baseline_sf1112 - total_energy_sf1112)/(total_energy_baseline_sf710 + total_energy_baseline_sf1112)*100 << std::endl;

    std::cerr << "total faults  = " << total_faults << std::endl;
    std::cerr << "total faults % = " << ((float) total_faults)/((float)total_upload)*100 << std::endl;
    std::cerr << "total energy saved = " << total_energy_baseline_sf710 + total_energy_sf1112 << std::endl;
    std::cerr << "total energy saved % = " << (total_energy_baseline_sf1112 - total_energy_sf1112)/(total_energy_baseline_sf710 + total_energy_baseline_sf1112)*100 << std::endl;

    int latency = 0;
    float total_ratio_baseline = 0.0;
    int total_real_upload_baseline = 0;
    int total_misses_baseline = 0;
    float total_ratio = 0.0;
    int total_real_upload = 0;
    int total_misses = 0;
	for (i = 0; i < VERTEX_NUM; i++)
	{
        while (!node[i].buffered_data.empty())
        {
            //std::cout << TOTAL_TICKS - node[i].buffered_data.front() << std::endl;
            total_real_upload ++;
            if ((TOTAL_TICKS - node[i].buffered_data.front()) > 180)
            {
                total_misses ++;
            }
            node[i].buffered_data.pop();
        }
        while (!node[i].baseline_buffered_data.empty())
        {
            if (node[i].sf >= 11)
            {
    std::cout << node[i].pid << " yyy " << clock_ticks - node[i].baseline_buffered_data.front() << std::endl;
                total_real_upload_baseline ++;
                if ((TOTAL_TICKS - node[i].baseline_buffered_data.front()) > 180)
                {
                    total_misses_baseline ++;
                }
            }
            else
            {
                total_real_upload_baseline ++;
                if ((TOTAL_TICKS - node[i].baseline_buffered_data.front()) > 180)
                {
                    total_misses_baseline ++;
                }
                total_real_upload ++;
                if ((TOTAL_TICKS - node[i].baseline_buffered_data.front()) > 180)
                {
                    total_misses ++;
                }
            }
            node[i].baseline_buffered_data.pop();
        }

        while (!node[i].latencies_baseline.empty())
        {
            total_real_upload_baseline ++;
            latency = node[i].latencies_baseline.front();
            if (latency > 180)
            {
                total_misses_baseline ++;
     std::cout << node[i].pid << " xxxx " << latency << std::endl;
            }
//          std::cout << node[i].pid << " " << latency << std::endl;
            node[i].latencies_baseline.pop ();
        }
        while (!node[i].latencies.empty())
        {
            total_real_upload ++;
            latency = node[i].latencies.front();
            if (latency > 180)
            {
                total_misses ++;
            }
//            std::cout << node[i].pid << " " << latency << std::endl;
            node[i].latencies.pop ();
        }
    }
//    std::cerr << "  total real upload baseline = " << total_real_upload_baseline << std::endl;
    total_ratio_baseline = ((float)total_misses_baseline)/((float)total_real_upload_baseline)*100;
    total_ratio = ((float)total_misses)/((float)total_real_upload)*100;

//    std::cerr << "total miss ratio (baseline) % = " << total_ratio_baseline << std::endl;
    std::cout << "total miss ratio (baseline) % = " << total_ratio_baseline << std::endl;
//    std::cerr << "total miss ratio % = " << total_ratio << std::endl;
    std::cout << "total miss ratio % = " << total_ratio << std::endl;
    std::cout << "-----------------------" << std::endl;

	for (i = 0; i < VERTEX_NUM; i++)
	{
        std::cout << "(Node " << node[i].pid << ") " << node[i].x << " " << node[i].y << " " << node[i].sf << " " << node[i].phase << std::endl;
    }

	for (i = 0; i < VERTEX_NUM; i++)
	{
        if (node[i].sf >= 11)
        {
            std::cout << "node " << node[i].pid << " trace = " << node[i].arrival_observed_history << ", uploads = " << node[i].upload_count << ", catches = " << node[i].catches << std::endl;
            std::cout << "node " << node[i].pid << " real = " << node[i].arrival_real_history << std::endl;
            free (node[i].arrival_observed_history);
            free (node[i].arrival_real_history);
        }
    }
*/
///*
    for (int i = 0; i < FN; i++)
    {
        delete faulty_comm[i];
    }
    delete faulty_comm;
//*/
	return 0;
}
