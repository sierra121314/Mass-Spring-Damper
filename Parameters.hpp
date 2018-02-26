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
    // 2ND ANTAGONIST //
    bool rand_antagonist;
    
    // DOMAIN VARIABLES - STATIC
    double m = 7;       //mass
    double b = 0.05;    //damper
    double k = 1;       //spring
    double dt = 0.1;    //time step [s]
    double mu = 0;      //friction
    bool MSD_EOM = true;
    bool Pend_EOM = false;
    
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
    int goal_x_upper_bound = 5;     //had to use int vs double due to rand() only works with ints
    int goal_x_lower_bound = 0;
    int start_x_upper_bound = 20;
    int start_x_lower_bound = 10;
    int start_x_dot_upper_bound = 5;
    int start_x_dot_lower_bound = 0;
    double A_g = 2;             //amplifier for goal sinusoidal
    bool sinusoidal_goal = false;
    double g_phase = 0;
    void moving_goal();
    bool multi_var;      //50 goals per policy
    void fifty_var();           //50 goals, start_x, start_x_dot
    vector<vector<int>> fifty_inits;
    
    // RANDOMIZING STARTS //
    bool rand_start_gen;
    bool rand_start_5gen;
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
    bool five_B;          //Antagonist that manipulates starting variables
    bool four_B;          //Primary with random starting variables per generation
    bool three_B;
    bool three_A;         //train test 3 combo
    bool two_B;           //Primary ...
    bool two_A;           //train test 2 combo
    bool one;           //train test 1 combo
    bool tr_1;          //pro plus ant with no noise
    bool tr_2;          // pro only
    bool tr_3;          //pro plus ant manipulating force
    bool tr_4;          // pro and random starting variables per gen
    bool tr_5;          //pro plus ant manipulating starting variables
    void train();
    bool te_1;          // tr_1 with noise
    bool te_A;          // Primary with no antagonist and noise
    bool te_B;          // Domain with Gaussian noise distribution on top of a sinusoidal wave and 50 sets of starting variables per policy
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
    start_x = start_x_lower_bound + double(rand() % start_x_upper_bound);
    goal_x  = goal_x_lower_bound + double(rand() % goal_x_upper_bound);
    start_x_dot = start_x_dot_lower_bound + double(rand() % start_x_dot_upper_bound);

}

void Parameters::test_train_set(){
    if (one ==true){
        tr_1 = true;
        te_1 = true;
        cout << "test one" <<endl;
    }
    if (two_A == true){
        tr_2 = true;
        te_A = true;
        cout << "train two - test A" <<endl;
    }
    if (two_B==true){
        tr_2 = true;
        te_B = true;
        cout << "train two - test B" <<endl;
    }
    if (three_A == true){
        tr_3 = true;
        te_A = true;
        cout << "train three - test A" <<endl;
    }
    if (three_B == true){
        tr_3 = true;
        te_B = true;
        cout << "train three - test B" <<endl;
    }
    if (four_B == true){
        tr_4 = true;    //Primary with random start per gen
        te_B = true;    //Primary with 50 starting variables per policy
        cout << "train four - test B" <<endl;
    }
    if (five_B == true){
        tr_5 = true;
        te_B = true;
        cout << "train five - test B" <<endl;
    }
}

void Parameters::train_para(){
    ofstream train_para;
    train_para.open("training_parameters.txt", ofstream::out | ofstream::trunc);
    
    train_para << "# Policies\t" << num_pol << "\t # Generations\t" << gen_max << "\t Mut Rate and Range\t" << mutation_rate << "\t" << mutate_range << endl;
    train_para << "Pro Bounds\t " << P_f_min_bound << "\t" << P_f_max_bound << endl;
    train_para << "Ant Bounds\t " << A_f_min_bound << "\t" << A_f_max_bound << endl;
    train_para << "Random Antagonist\t" << rand_antagonist << endl;
    train_para << "x and xdot Bounds\t " << x_min_bound << "\t" << x_max_bound << "\t" << x_dot_min_bound << "\t" << x_dot_max_bound << endl;
    train_para << "# NN Input-Output-Nodes\t" << num_inputs << "\t" << num_outputs << "\t" << num_nodes << endl;
    train_para << "SENSOR Noise\t" << sensor_NOISE << "\t ACTUATOR Noise" << actuator_NOISE << endl << "SINUSOIDAL Noise (if sensor or actuator is true)" << sinusoidal_noise << "\tPHASE" << phase << endl;
    train_para << "Random Starts/Gen\t" << rand_start_gen << "\t" << rand_start_5gen << endl;
    train_para << "Reverse Leniency\t" << three_for_three << endl;
    train_para << "50 Starting Variables/Policy\t" << multi_var << endl;
    train_para << "goal upper and lower bound\t" << goal_x_upper_bound << "\t" << goal_x_lower_bound << endl;
     train_para << "start_x upper and lower bound\t" << start_x_upper_bound << "\t" << start_x_lower_bound << endl;
    train_para << "start_xdot upper and lower bound\t" << start_x_dot_upper_bound << "\t" << start_x_dot_lower_bound << endl;
    train_para.close();
}

void Parameters::train(){
    if (train_and_test == true){
        if (tr_1 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -1;
            A_f_max_bound = 1;
            rand_antagonist = false;
            sensor_NOISE = false;
            actuator_NOISE = false;
            rand_start_gen = false;
            rand_start_5gen = false;
            multi_var = false; //do NOT change this one
            
        }
        if (tr_2 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -0;
            A_f_max_bound = 0;
            rand_antagonist = false;
            sensor_NOISE = false;
            actuator_NOISE = false;
            rand_start_gen = false;
            rand_start_5gen = false;
            multi_var = false; //do NOT change this one
        }
        if (tr_3 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -1;
            A_f_max_bound = 1;
            rand_antagonist = false;
            sensor_NOISE = false;
            actuator_NOISE = false;
            rand_start_gen = false;
            rand_start_5gen = false;
            multi_var = false; //do NOT change this one
        }
        if (tr_4 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -0;
            A_f_max_bound = 0;
            rand_antagonist = false;
            sensor_NOISE = false;
            actuator_NOISE = false;
            rand_start_gen = true; //pick one or the other
            rand_start_5gen = false;
            multi_var = false; //do NOT change this one
        }
        if (tr_5 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -0;
            A_f_max_bound = 0;
            rand_antagonist = true;
            sensor_NOISE = false;
            actuator_NOISE = false;
            rand_start_gen = false;
            rand_start_5gen = false;
            multi_var = false; //do NOT change this one

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
    test_para << "Random Antagonist\t" << rand_antagonist << endl;
    test_para << "x and xdot Bounds\t " << x_min_bound << "\t" << x_max_bound << "\t" << x_dot_min_bound << "\t" << x_dot_max_bound << endl;
    test_para << "# NN Input-Output-Nodes\t" << num_inputs << "\t" << num_outputs << "\t" << num_nodes << endl;
    test_para << "SENSOR Noise\t" << sensor_NOISE << "\t ACTUATOR Noise" << actuator_NOISE << endl <<  "SINUSOIDAL Noise (if sensor or actuator is true)" << sinusoidal_noise << "\tPHASE" << phase << endl;
    test_para << "Random Starts/Gen\t" << rand_start_gen << "\t" << rand_start_5gen << endl;
    test_para << "Reverse Leniency\t" << three_for_three << endl;
    test_para << "50 Starting Variables/Policy\t" << multi_var << endl;
    test_para << "goal upper and lower bound\t" << goal_x_upper_bound << "\t" << goal_x_lower_bound << endl;
    test_para << "start_x upper and lower bound\t" << start_x_upper_bound << "\t" << start_x_lower_bound << endl;
    test_para << "start_xdot upper and lower bound\t" << start_x_dot_upper_bound << "\t" << start_x_dot_lower_bound << endl;
    
    test_para.close();
}

void Parameters::test(){
    if (train_and_test == true){
        if (te_1 == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -1;
            A_f_max_bound = 1;
            rand_antagonist = false;
            sensor_NOISE = true;
            actuator_NOISE = true;
            multi_var = false;
            three_for_three = false; //do NOT change this one
            rand_start_gen = false;  //do NOT change this one
        }
        if (te_A == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -0;
            A_f_max_bound = 0;
            rand_antagonist = false;
            sensor_NOISE = true;
            actuator_NOISE = true;
            multi_var = false;       //50 rand variables per policy
            three_for_three = false; //do NOT change this one
            rand_start_gen = false;  //do NOT change this one
        }
        if (te_B == true){
            P_f_min_bound = -5;
            P_f_max_bound = 5;
            A_f_min_bound = -0;
            A_f_max_bound = 0;
            rand_antagonist = false;
            sensor_NOISE = true;
            actuator_NOISE = true;
            multi_var = true;       //50 rand variables per policy
            three_for_three = false; //do NOT change this one
            rand_start_gen = false; //do NOT change this one
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
            three_inits.push_back(goal_x_lower_bound+double(rand() % goal_x_upper_bound));//goal_x(0to6)
            three_inits.push_back(start_x_lower_bound + double(rand()%start_x_upper_bound));//start_x=something;(0 to 25) //
            three_inits.push_back(start_x_dot_lower_bound + double(rand() % start_x_dot_upper_bound));//start_x_dot=something;(0 to 5)
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
