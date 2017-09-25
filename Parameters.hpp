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
    int gen_max = 100;                   //number of generations
    double mutation_rate = 0.5;         //mutation rate
    double mutate_range = 0.1;          //mutation range
    
    int num_weights;
    
    int num_inputs = 2;
    int num_outputs = 1;
    int num_nodes = 10;
    
    //CHANGE THESE VARIABLES
    double start_x = 15;
    double start_x_dot = 0;
    double start_x_dd = 0;
    double start_P_force = 0;
    double start_A_force = 0;
    double displace = 2;
    
    // DOMAIN VARIABLES - STATIC
    double m = 7;       //mass
    double b = 1;      //damper
    double k = 1;       //spring
    double dt = 0.1;    //time step [s]
    double mu = 0;      //friction
    
    double P_force;     //Protagonist force
    double A_force;     //Antagonist force
    
    //double P_desired_x = 0;
    double P_desired_x_dot = 0;
    double P_desired_x_dd = 0;
    
    double total_time = 500; //total time
    
    // NN BOUNDARIES //
    double P_f_min_bound = -0;
    double P_f_max_bound = 0;
    double A_f_min_bound = -5;
    double A_f_max_bound = 5;
    double x_min_bound = 0;
    double x_max_bound = 20;
    double x_dot_min_bound = -.2;
    double x_dot_max_bound = .2;
    //double x_dd_min_bound = -6;
    //double x_dd_max_bound = 6;
    
    double w1 = 1;
    double w2 = 10;
    
    
    bool only_pro = true;    //Just protagonist?
    bool sensor_NOISE = true;
    bool actuator_NOISE = false;
    
    
private:
};

#endif /* Parameters_hpp */
