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
#define PI 3.1415926535897

using namespace std;


class Parameters
{
    friend class EA;
    friend class Simulator;
    friend class Policy;
    
protected:
    
    
public:
    // EA STUFF //
    int num_pol = 100;                  //number of policies
    int to_kill = num_pol/2;
    int gen_max = 500;                  //number of generations
    double total_time = 1000;            //total time steps
    double mutation_rate = 0.5;         //mutation rate
    double mutate_range = 0.1;          //mutation range
    
    double P_force;                     //Protagonist force
    double A_force;                     //Antagonist force
    
    // DOMAIN VARIABLES - STATIC
    double m = 7;       //mass
    double b = 0.05;    //damper
    double k = 1;       //spring
    double dt = 0.1;    //time step [s]
    double mu = 0;      //friction
    
    // NEURAL NETWORK STUFF //
    int num_weights;
    int num_inputs = 2;
    int num_outputs = 1;
    int num_nodes = 10;
    
    // GOAL VARIABLES //
    double start_x = 15;
    double start_x_dot = 0;
    double start_x_dd = 0;
    double start_P_force = 0;
    double start_A_force = 0;
    double displace = 2;        //initial displacement
    double goal_x;              //ending position (start_x+goal_x);
    double A_g = 2;             //amplifier for goal sinusoidal
    bool sinusoidal_goal = false;
    double g_phase = 0;
    void moving_goal();
    bool multi_var;      //50 goals per policy
    void fifty_var();           //50 goals, start_x, start_x_dot
    vector<vector<int>> fifty_inits;
    
    // 2ND ANTAGONIST //
    bool rand_antagonist = false;
    
    // RANDOMIZING STARTS //
    bool rand_start_gen = false;
    bool rand_start_5gen = false;
    void random_variables();
    
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

    // FITNESS WEIGHTS //
    double w1 = 1;      //Weight on Pro and Ant
    double w2 = 1;      //weight on extra penalty
    
    // SIMULATION NOISE //
    bool only_pro = true;           //Just protagonist?
    bool sensor_NOISE = false;      //Determined by train-test otherwise default
    bool actuator_NOISE = false;    //Determined by train-test otherwise default
    double sn = 1;                  //sensor noise magnitude
    double an = 1;                  //actuator noise magnitude
    double As = 2;
    bool sinusoidal_noise = true;   //within sensor and actuator noise, so if those are false->sinusoidal is false
    double phase = PI/4;            //in Radians
    
    
    // TRAINING AND TESTING MODES //
    bool train_and_test;
    void test_train_set();
    bool three;         //train test 3 combo
    bool two;           //train test 2 combo
    bool one;           //train test 1 combo
    bool tr_1=false;    //pro plus ant with no noise
    bool tr_2=false;    // pro only
    bool tr_3=false;    //pro plus ant
    void train();
    bool te_1=false;    // tr_1 with noise
    bool te_2=false;    // tr_2 with noise
    bool te_3=false;    // noise and no ANT
    void test();

    bool three_for_three;   //Reverse Leniancy

    // GRAPHS //
    void train_para();
    void test_para();
    
private:
};

void Parameters::random_variables(){
    
    // DOMAIN VARIABLES - STATIC
    //m = 7 + rand() % 2;       //mass
    //b = 1 + rand() % 2;       //damper
    //k = 1 + rand() % 2;       //spring
    //mu = 0 + rand() % 2;      //friction
    start_x = 5 + rand() % 5;
    goal_x  = rand() % 5;
    start_x_dot = 0 + rand() % 5;  

}

void Parameters::test_train_set(){
    if (one ==true){
        tr_1 = true;
        te_1 = true;
        cout << "test one" <<endl;
    }
    if (two == true){
        tr_2 = true;
        te_2 = true;
        cout << "test two" <<endl;
    }
    if (three == true){
        tr_3 = true;
        te_3 = true;
        cout << "test three" <<endl;
    }
}

void Parameters::train_para(){
    ofstream train_para;
    train_para.open("training_parameters.txt", ofstream::out | ofstream::trunc);
    
    train_para << "# Policies\t" << num_pol << "\t # Generations\t" << gen_max << "\t Mut Rate and Range\t" << mutation_rate << "\t" << mutate_range << endl;
    train_para << "Pro Bounds\t " << P_f_min_bound << "\t" << P_f_max_bound << endl;
    train_para << "Ant Bounds\t " << A_f_min_bound << "\t" << A_f_max_bound << endl;
    train_para << "x and xdot Bounds\t " << x_min_bound << "\t" << x_max_bound << "\t" << x_dot_min_bound << "\t" << x_dot_max_bound << endl;
    train_para << "# NN Input-Output-Nodes\t" << num_inputs << "\t" << num_outputs << "\t" << num_nodes << endl;
    train_para << "Noise\t" << sensor_NOISE << "\t" << actuator_NOISE << "\t" << sinusoidal_noise << "\t" << phase << endl;
    train_para << "Random Starts/Gen\t" << rand_start_gen << endl;
    train_para << "Reverse Leniency\t" << three_for_three << endl;
    train_para << "50 Goals/Policy\t" << multi_var << endl;
    train_para.close();
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
        train_para();
        
    }
}

void Parameters::test_para(){
    ofstream test_para;
    test_para.open("testing_parameters.txt", ofstream::out | ofstream::trunc);
    
    test_para << "# Policies\t" << num_pol << "\t # Generations\t" << gen_max << "\t Mut Rate and Range\t" << mutation_rate << "\t" << mutate_range << endl;
    test_para << "Pro Bounds\t " << P_f_min_bound << "\t" << P_f_max_bound << endl;
    test_para << "Ant Bounds\t " << A_f_min_bound << "\t" << A_f_max_bound << endl;
    test_para << "x and xdot Bounds\t " << x_min_bound << "\t" << x_max_bound << "\t" << x_dot_min_bound << "\t" << x_dot_max_bound << endl;
    test_para << "# NN Input-Output-Nodes\t" << num_inputs << "\t" << num_outputs << "\t" << num_nodes << endl;
    test_para << "Noise\t" << sensor_NOISE << "\t" << actuator_NOISE << "\t" << sinusoidal_noise << "\t" << phase << endl;
    test_para << "Random Starts/Gen\t" << rand_start_gen << endl;
    test_para << "Reverse Leniency\t" << three_for_three << endl;
    test_para << "50 Goals/Policy\t" << multi_var << endl;
    
    test_para.close();
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
        test_para();
    }
}

void Parameters::fifty_var(){
    if (multi_var==true) {
        fstream fifty_history;
        fifty_history.open("fiftysets_init.txt", ofstream::out | ofstream::trunc);
        for (int i=0; i<50; i++) {
            vector<int> three_inits;
            
            //Initialize 50x3 variables
            three_inits.push_back(rand() % 6);//goal_x(0to6)
            three_inits.push_back(5 + rand() % 25);//start_x=something;(0 to 25)
            three_inits.push_back(0 + rand() % 5);//start_x_dot=something;(0 to 5)
            for (int j=0; j<3; j++) {
                fifty_history << three_inits.at(j) << "\t";
            }
            fifty_history << endl;
            fifty_inits.push_back(three_inits);
        }
        fifty_history.close();
    }
}

void Parameters::moving_goal(){
    if (sinusoidal_goal==true){
        //g_xt = g_xt+dt;
        //goal_x = sin(2*PI*(g_xt+dt)+phase);
        
    }
}


#endif /* Parameters_hpp */
