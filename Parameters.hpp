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
    int gen_max = 100;                  //number of generations
    double total_time = 1000;            //total time steps
    double mutation_rate = 0.5;         //mutation rate
    double mutate_range = 0.1;          //mutation range
    
    // NEURAL NETWORK STUFF
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
    double goal_x = 2; //ending position (start_x+goal_x);
    
    // DOMAIN VARIABLES - STATIC
    double m = 7;       //mass
    double b = 0.05;      //damper
    double k = 1;       //spring
    double dt = 0.1;    //time step [s]
    double mu = 0;      //friction
    
    // RANDOMIZING STARTS //
    bool rand_start_ts = false;
    bool rand_start_gen = false;
    void random_variables();
    
    double P_force;     //Protagonist force
    double A_force;     //Antagonist force
    
    //double P_desired_x = 0;
    double P_desired_x_dot = 0;
    double P_desired_x_dd = 0;
    
    // WHAT TO GRAPH
    bool best_v_median = true;
    
    
    // NN BOUNDARIES //
    double P_f_min_bound; //set in main
    double P_f_max_bound;
    double A_f_min_bound;
    double A_f_max_bound;
    double x_min_bound = 0;
    double x_max_bound = 20;
    double x_dot_min_bound = -.2;
    double x_dot_max_bound = .2;

    
    double w1 = 1;
    double w2 = 10;
    
    // NOISE is put into simulation //
    bool only_pro = true;       //Just protagonist?
    bool sensor_NOISE = false;
    bool actuator_NOISE = false;
    double sn = 1;              //sensor noise magnitude
    double an = 1;              //actuator noise magnitude
    bool sinusoidal_noise = true;
    double phase = 3.1415/2;    //in Radians
    
    bool train_and_test;
    bool tr_1=false;    //pro plus ant with no noise
    bool tr_2=false;    // pro only
    bool tr_3=false;    //pro plus ant
    void train();
    bool te_1=false;    // tr_1 with noise
    bool te_2=false;    // tr_2 with noise
    bool te_3=false;    // noise and no ANT
    void test();


    
private:
};

void Parameters::random_variables(){
    if (rand_start_ts == true) {
        
        // DOMAIN VARIABLES - STATIC
        m = 7 + rand() % 2;       //mass
        b = 1 + rand() % 2;       //damper
        k = 1 + rand() % 2;       //spring
        mu = 0 + rand() % 2;      //friction
        start_x = 15 + rand() % 2;
    }
    
    if (rand_start_gen == true){
        // DOMAIN VARIABLES - STATIC
        m = 7 + rand() % 2;       //mass
        b = 1 + rand() % 2;       //damper
        k = 1 + rand() % 2;       //spring
        mu = 0 + rand() % 2;      //friction
        start_x = 15 + rand() % 2;
    }
}

void Parameters::train(){
    if (train_and_test == true){
        if (tr_1 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -1;
            A_f_max_bound = 1;
            sensor_NOISE = false;
            actuator_NOISE = false;

        }
        if (tr_2 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -0;
            A_f_max_bound = 0;
            sensor_NOISE = false;
            actuator_NOISE = false;
        }
        if (tr_3 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -1;
            A_f_max_bound = 1;
            sensor_NOISE = false;
            actuator_NOISE = false;
        }
    }
}

void Parameters::test(){
    if (train_and_test == true){
        if (te_1 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -1;
            A_f_max_bound = 1;
            sensor_NOISE = true;
            actuator_NOISE = true;
            
        }
        if (te_2 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -0;
            A_f_max_bound = 0;
            sensor_NOISE = true;
            actuator_NOISE = true;
        }
        if (te_3 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -0;
            A_f_max_bound = 0;
            sensor_NOISE = true;
            actuator_NOISE = true;
        }
    }
}

#endif /* Parameters_hpp */
