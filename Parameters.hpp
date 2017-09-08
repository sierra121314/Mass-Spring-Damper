//
//  Parameters.hpp
//  Pro-Ant_EA_project
//
//  Created by Sierra Gonzales on 8/21/17.
//  Copyright © 2017 Pro-Ant_EA_project. All rights reserved.
//

#ifndef Parameters_hpp
#define Parameters_hpp

#include <stdio.h>

using namespace std;


class Parameters
{
    friend class EA;
    friend class Simulator;
    friend class Policy;
    
protected:
    
    
public:
    int num_pol = 100;                  //number of policies
    int to_kill = num_pol/2;
    int gen_max = 50;                   //number of generations
    double mutation_rate = 0.5;         //mutation rate
    double mutate_range = 0.05;          //mutation range
    
    int num_weights;
    
    int num_inputs = 3;
    int num_outputs = 1;
    int num_nodes = 10;
    
    double start_x = 15;
    double start_x_dot = 0;
    double start_x_dd = 0;
    double start_P_force = 0;
    
    int f_min_bound = -10;
    int f_max_bound = 10;
    int x_min_bound = 0;
    int x_max_bound = start_x*3;
    int x_dot_min_bound = -175;
    int x_dot_max_bound = 175;
    int x_dd_min_bound = -6;
    int x_dd_max_bound = 6;
    
    
    bool only_pro = true;    //Just protagonist?
    
    
    
private:
};

#endif /* Parameters_hpp */
