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
    int num_pol = 20;                  //number of policies
    int to_kill = num_pol/2;
    int gen_max = 500;                  //number of generations
    double total_time = 1000;            //total time steps
    double mutation_rate = 0.5;         //mutation rate
    double mutate_range = 0.1;          //mutation range
    
    // DOMAIN VARIABLES - STATIC
    double m = 7;       //mass
    double b = 0.05;    //damper
    double k = 1;       //spring
    double dt = 0.1;    //time step [s]
    double mu = 1;      //friction
    bool MSD_EOM = false;
    bool Pend_EOM = true;
    bool full_leniency = false;
    
    // MSD DOMAIN
    double msd_start = 15;
    double init_msd_goal = 2;
    
    // PENDULUM DOMAIN
    double L = 10;
    double pend_m = 0.25;                  //pendulum mass - set in reset_var
    double init_pend_goal = 0;
    double pend_start = ((-5)*PI/180);
    double pend_goal;
    
    double P_force;                     //Protagonist force
    double A_force;                     //Antagonist force
    double start_P_force = 0;
    double start_A_force = 0;
    
    // NEURAL NETWORK STUFF //
    int num_weights;
    int A_num_weights;
    int A_num_inputs = 2;
    int num_inputs = 2;
    int num_outputs = 1;
    int num_nodes = 10;
    
    // NN BOUNDARIES //
    double P_f_min_bound; //set in main
    double P_f_max_bound;
    double A_f_min_bound;
    double A_f_max_bound;
    double x_min_bound;
    double x_max_bound;       //maybe this should be max boundary plus goal as max
    double x_dot_min_bound;
    double x_dot_max_bound;
    
    
    // GOAL VARIABLES //
    void reset_start_var();
    double init_start_x;
    double init_start_x_dot = 0;
    double start_x;
    double start_x_dot;
    double start_x_dd = 0;
    
    double init_displace = 2;        //initial displacement
    double displace;
    double init_goal_x;
    double goal_x;                      //ending position (start_x+goal_x);
    
    
    
    double goal_x_upper_bound = 2;
    double goal_x_lower_bound = -2;
    double start_x_upper_bound = 20;
    double start_x_lower_bound = 5;
    double start_x_dot_upper_bound = 2;
    double start_x_dot_lower_bound = -2;
    double displace_upper_bound = 2;
    double displace_lower_bound = -2;
    
    double ant_goal_x_upper_bound = goal_x_upper_bound;
    double ant_goal_x_lower_bound = goal_x_lower_bound;
    double ant_start_x_upper_bound = start_x_upper_bound;
    double ant_start_x_lower_bound = start_x_lower_bound;
    double ant_start_x_dot_upper_bound = start_x_dot_upper_bound;
    double ant_start_x_dot_lower_bound = start_x_dot_lower_bound;
    double ant_displace_upper_bound = displace_upper_bound;
    double ant_displace_lower_bound = displace_lower_bound;
    
    
    
    
    // RANDOMIZING STARTS //
    bool rand_start_gen;
    bool rand_start_5gen;
    void random_start_end_variables();
    
    // 2ND ANTAGONIST //
    bool rand_antagonist = false;
    bool rand_ant_per_5gen = false;         //Allows variable manipulating ANT to evolve every 5 generations
    double num_loops;
    vector<double> fifty_fitness;
    bool multi_var;                         //50 goals per policy
    void fifty_var();                       //50 goals, start_x, start_x_dot
    vector<vector<double>> fifty_inits;
    
    
    // WHAT TO GRAPH
    bool best_v_median = true;
    
    // FITNESS WEIGHTS //
    double w1 = 1;      //Weight on Pro and Ant
    double w2 = 1;      //weight on extra penalty
    
    // SIMULATION NOISE //
    bool sensor_NOISE;      //Determined by train-test otherwise default
    bool actuator_NOISE;    //Determined by train-test otherwise default
    bool sinusoidal_noise = true;   //within sensor and actuator noise, so if those are false->sinusoidal is false
    double sn = 1;                  //sensor noise magnitude
    double an = 1;                  //actuator noise magnitude
    double As = 1;                  //Sinusoidal Noise Magnitude
    double phase = PI/4;            //in Radians
    double lambda = 0.05;              //Period - 2PI when lambda is 1
    
    // TRAINING AND TESTING MODES //
    void train_set();       //Depending on combo selected, set testing bool to true
    void test_set();        //Depending on combo selected, set testing bool to true
    
    bool five_A;
    bool five_B;          //Antagonist that manipulates starting variables
    bool four_A;
    bool four_B;          //Primary with random starting variables per generation
    bool three_B;
    bool three_A;         //train test 3 combo
    bool two_B;           //Primary ...
    bool two_A;           //train test 2 combo
    
    bool tr_2 = false;          // pro only
    bool tr_3 = false;          //pro plus ant manipulating force
    bool tr_4 = false;          // pro and random starting variables per gen
    bool tr_5 = false;          //pro plus ant manipulating starting variables
    void train();

    bool te_A = false;          // Primary with no antagonist and noise
    bool te_B = false;          // Domain with Gaussian noise distribution on top of a sinusoidal wave and 50 sets of starting variables per policy
    void test();


    // GRAPHS //
    void train_para();
    void test_para();
    void trunc_Graphs();
    
private:
};


void Parameters::reset_start_var(){
    //init_start_x = 10 + (rand() % 20);
    
    start_x_dot = init_start_x_dot;
    start_x_dd = 0;
    
    if (MSD_EOM==true){
        init_start_x = msd_start;
        init_goal_x = init_msd_goal;
        displace = init_displace;
        
        x_min_bound = 0;
        x_max_bound = 30;        //maybe this should be max boundary plus goal as max
        x_dot_min_bound = -.2;
        x_dot_max_bound = .2;
    }
    else if(Pend_EOM==true){
        init_goal_x = init_pend_goal;
        pend_goal = init_pend_goal;
        init_start_x = pend_start;
        init_displace = 0;
        m = pend_m;
        
        x_min_bound = -2*PI/2;
        x_max_bound = 2*PI/2;        //maybe this should be max boundary plus goal as max
        x_dot_min_bound = -2;
        x_dot_max_bound = 2;
    }
    
    start_x = init_start_x;
    goal_x = init_goal_x;
    displace = init_displace;
    //init_goal_x  = 0 + (rand() % 5);
    
    assert((Pend_EOM==true && m == pend_m) || (MSD_EOM==true && m == m));
    
}

void Parameters::random_start_end_variables(){
    
    // DOMAIN VARIABLES - STATIC
    //m = 7 + rand() % 2;       //mass
    //b = 1 + rand() % 2;       //damper
    //k = 1 + rand() % 2;       //spring
    //mu = 0 + rand() % 2;      //friction
    start_x = start_x_lower_bound + double(rand() % (int)(start_x_upper_bound-start_x_lower_bound));
    goal_x  = goal_x_lower_bound + double(rand() % (int)(goal_x_upper_bound-goal_x_lower_bound));
    start_x_dot = start_x_dot_lower_bound + double(rand() % (int)(start_x_dot_upper_bound-start_x_dot_lower_bound));
    displace = displace_lower_bound + double(rand() % (int)(displace_upper_bound-displace_lower_bound));

    assert(start_x <= start_x_upper_bound && start_x >= start_x_lower_bound);
    assert(start_x_dot <= start_x_dot_upper_bound && start_x_dot >= start_x_dot_lower_bound);
    assert(goal_x <= goal_x_upper_bound && goal_x >= goal_x_lower_bound);
    assert(displace <= displace_upper_bound && displace >= displace_lower_bound);
}

void Parameters::train_set(){
    reset_start_var();
    if (two_A == true){
        tr_2 = true;
        assert(tr_3==false && tr_4==false && tr_5==false);
        cout << "train two - test A" <<endl;
    }
    if (two_B==true){
        tr_2 = true;
        assert(tr_3==false && tr_4==false && tr_5==false);
        cout << "train two - test B" <<endl;
    }
    if (three_A == true){
        tr_3 = true;
        assert(tr_2==false && tr_4==false && tr_5==false);
        cout << "train three - test A" <<endl;
    }
    if (three_B == true){
        tr_3 = true;
        assert(tr_2==false && tr_4==false && tr_5==false);
        cout << "train three - test B" <<endl;
    }
    if (four_A == true){
        tr_4 = true;    //Primary with random start per gen
        assert(tr_2==false && tr_3==false && tr_5==false);
        cout << "train four - test A" <<endl;
    }
    if (four_B == true){
        tr_4 = true;    //Primary with random start per gen
        assert(tr_2==false && tr_3==false && tr_5==false);
        cout << "train four - test B" <<endl;
    }
    if (five_A == true){
        tr_5 = true;
        assert(tr_2==false && tr_3==false && tr_4==false);
        cout << "train five - test A" <<endl;
    }
    if (five_B == true){
        tr_5 = true;
        assert(tr_2==false && tr_3==false && tr_4==false);
        cout << "train five - test B" <<endl;
    }
    te_A=false;
    te_B=false;
}

void Parameters::test_set(){
    
    if (two_A == true){
        te_A = true;
        assert(te_B==false);
        cout << "Start - test A" <<endl;
    }
    if (two_B==true){
        te_B = true;
        assert(te_A==false);
        cout << "Start - test B" <<endl;
    }
    if (three_A == true){
        te_A = true;
        assert(te_B==false);
        cout << "Start - test A" <<endl;
    }
    if (three_B == true){
        te_B = true;
        assert(te_A==false);
        cout << "Start - test B" <<endl;
    }
    if (four_A == true){
        te_A = true;
        assert(te_B==false);
        cout << "Start - test A" <<endl;
    }
    if (four_B == true){
        te_B = true;    //Primary with 50 starting variables per policy
        assert(te_A==false);
        cout << "Start - test B" <<endl;
    }
    if (five_A == true){
        te_A = true;
        assert(te_B==false);
        cout << "Start - test A" <<endl;
    }
    if (five_B == true){
        te_B = true;
        assert(te_A==false);
        cout << "Start - test B" <<endl;
    }
    tr_2=false;
    tr_3=false;
    tr_4=false;
    tr_5=false;
}


void Parameters::train_para(){
    ofstream train_para;
    train_para.open("training_parameters.txt", ofstream::out | ofstream::trunc);
    
    train_para << "# Policies\t" << num_pol << "\t # Generations\t" << gen_max << "\t Mut Rate and Range\t" << mutation_rate << "\t" << mutate_range << endl;
    train_para << "Pro Bounds\t " << P_f_min_bound << "\t" << P_f_max_bound << endl;
    train_para << "Ant Bounds\t " << A_f_min_bound << "\t" << A_f_max_bound << endl;
    train_para << "Random Antagonist\t" << rand_antagonist << "\tEvery 5 gen\t" << rand_ant_per_5gen << endl;
    train_para << "x and xdot Bounds\t " << x_min_bound << "\t" << x_max_bound << "\t" << x_dot_min_bound << "\t" << x_dot_max_bound << endl;
    train_para << "# NN Input-Output-Nodes\t" << num_inputs << "\t" << num_outputs << "\t" << num_nodes << endl;
    train_para << "SENSOR Noise\t" << sensor_NOISE << "\t ACTUATOR Noise" << actuator_NOISE << endl << "SINUSOIDAL Noise (if sensor or actuator is true)" << sinusoidal_noise << "\tPHASE" << phase << "\tFrequency" << lambda << endl;
    
    train_para << "Random Starts/Gen\t" << rand_start_gen << "\t" << rand_start_5gen << endl;
    train_para << "50 Starting Variables/Policy\t" << multi_var << endl;
    train_para << "goal upper and lower bound\t" << goal_x_upper_bound << "\t" << goal_x_lower_bound << endl;
     train_para << "start_x upper and lower bound\t" << start_x_upper_bound << "\t" << start_x_lower_bound << endl;
    train_para << "start_xdot upper and lower bound\t" << start_x_dot_upper_bound << "\t" << start_x_dot_lower_bound << endl;
    train_para << "displacement upper and lower bound\t" << displace_upper_bound << "\t" << displace_lower_bound << endl;
    
    train_para << "ANT goal upper and lower bound\t" << ant_goal_x_upper_bound << "\t" << ant_goal_x_lower_bound << endl;
    train_para << "ANT start_x upper and lower bound\t" << ant_start_x_upper_bound << "\t" << ant_start_x_lower_bound << endl;
    train_para << "ANT start_xdot upper and lower bound\t" << ant_start_x_dot_upper_bound << "\t" << ant_start_x_dot_lower_bound << endl;
    train_para << "ANT displacement upper and lower bound\t" << ant_displace_upper_bound << "\t" << ant_displace_lower_bound << endl;
    
    train_para.close();
}

void Parameters::train(){
    sensor_NOISE = false;      //Determined by train-test otherwise default
    actuator_NOISE = false;    //Determined by train-test otherwise default
    
    if (tr_2 == true){
        P_f_min_bound = -5;
        P_f_max_bound = 5;
        A_f_min_bound = -0;
        A_f_max_bound = 0;
        
        rand_antagonist = false;
        rand_ant_per_5gen = false;
        rand_start_gen = false;
        rand_start_5gen = false;
        multi_var = false;          //do NOT change this one
    }
    if (tr_3 == true){
        P_f_min_bound = -5;
        P_f_max_bound = 5;
        A_f_min_bound = -1;
        A_f_max_bound = 1;
        
        rand_antagonist = false;
        rand_ant_per_5gen = false;
        rand_start_gen = false;
        rand_start_5gen = false;
        multi_var = false;          //do NOT change this one
    }
    if (tr_4 == true){
        P_f_min_bound = -5;
        P_f_max_bound = 5;
        A_f_min_bound = -0;
        A_f_max_bound = 0;
        
        rand_antagonist = false;
        rand_ant_per_5gen = false;
        rand_start_gen = true; //pick one or the other
        rand_start_5gen = false;
        multi_var = false;          //do NOT change this one
    }
    if (tr_5 == true){
        P_f_min_bound = -5;
        P_f_max_bound = 5;
        A_f_min_bound = -0;
        A_f_max_bound = 0;
        
        rand_antagonist = true;
        rand_ant_per_5gen = false;       //turn on or off
        rand_start_gen = false;
        rand_start_5gen = false;
        multi_var = false;          //do NOT change this one
        
    }
    train_para();
    
    
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
    test_para << "SENSOR Noise\t" << sensor_NOISE << "\t ACTUATOR Noise" << actuator_NOISE << endl << "SINUSOIDAL Noise (if sensor or actuator is true)" << sinusoidal_noise << "\tPHASE" << phase << "\tFrequency" << lambda << endl;
    test_para << "Random Starts/Gen\t" << rand_start_gen << "\t" << rand_start_5gen << endl;
    test_para << "50 Starting Variables/Policy\t" << multi_var << endl;
    test_para << "goal upper and lower bound\t" << goal_x_upper_bound << "\t" << goal_x_lower_bound << endl;
    test_para << "start_x upper and lower bound\t" << start_x_upper_bound << "\t" << start_x_lower_bound << endl;
    test_para << "start_xdot upper and lower bound\t" << start_x_dot_upper_bound << "\t" << start_x_dot_lower_bound << endl;
    test_para << "displacement upper and lower bound\t" << displace_upper_bound << "\t" << displace_lower_bound << endl;
    
    test_para.close();
}

void Parameters::test(){
    
    if (te_A == true){
        P_f_min_bound = -5;
        P_f_max_bound = 5;
        A_f_min_bound = -0;
        A_f_max_bound = 0;

        sensor_NOISE = true;
        actuator_NOISE = true;
        
        rand_antagonist = false;
        multi_var = false;       //50 rand variables per policy
        rand_start_gen = false;  //do NOT change this one
        rand_ant_per_5gen = false;
    }
    if (te_B == true){
        P_f_min_bound = -5;
        P_f_max_bound = 5;
        A_f_min_bound = -0;
        A_f_max_bound = 0;
        
        sensor_NOISE = true;       //Changed from True to test
        actuator_NOISE = true;      // Changed from True to test
        
        rand_antagonist = false;
        multi_var = true;               //50 rand variables per policy
        rand_start_gen = false;         //do NOT change this one
        rand_ant_per_5gen = false;      //do NOT change this one
        
        fifty_var();
    }
    test_para();
    
}

void Parameters::fifty_var(){
    if (multi_var==true) {
        fstream fifty_history;
        fifty_history.open("fiftysets_init.txt", fstream::app);
        fifty_inits.clear();
        for (int i=0; i<50; i++) {
            vector<double> three_inits;
            
            //Initialize 50x3 variables
            three_inits.push_back(goal_x_lower_bound + double(rand() % (int)(goal_x_upper_bound-goal_x_lower_bound)));              //goal_x(0to5)  //actually goal value is this plus the start_x of previous
            three_inits.push_back(start_x_lower_bound + double(rand() % (int)(start_x_upper_bound-start_x_lower_bound)));          //start_x=something;(0 to 25) //
            three_inits.push_back(start_x_dot_lower_bound + double(rand() % (int)(start_x_dot_upper_bound-start_x_dot_lower_bound)));  //start_x_dot=something;(0 to 5)
            three_inits.push_back(displace_lower_bound + double(rand() % (int)(displace_upper_bound-displace_lower_bound)));
            
            assert(three_inits.at(0)>=goal_x_lower_bound && three_inits.at(0)<=goal_x_upper_bound);
            assert(three_inits.at(1)>=start_x_lower_bound && three_inits.at(1)<=start_x_upper_bound);
            assert(three_inits.at(2)>=start_x_dot_lower_bound && three_inits.at(2)<=start_x_dot_upper_bound);
            assert(three_inits.at(3)>=displace_lower_bound && three_inits.at(3)<=displace_upper_bound);
            
            for (int j=0; j<4; j++) {
                fifty_history << three_inits.at(j) << "\t";
            }
            fifty_history << endl;
            
            assert(three_inits.size()==4);
            fifty_inits.push_back(three_inits);
        }
        assert(fifty_inits.size()==50);
        fifty_history << "___________________________________________" << endl;
        fifty_history.close();
    }
}


void Parameters::trunc_Graphs(){
    // TRAIN //
    ofstream x_best,xdot_best,xdd_best, best_P_force,best_A_force, P_fit_hist, A_fit_hist, SR, SR_A,P_fit,A_fit;
    x_best.open("x_history.txt", std::ofstream::out | ofstream::trunc);
    xdot_best.open("x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    xdd_best.open("x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    best_P_force.open("P_force_history.txt",std::ofstream::out | ofstream::trunc);
    best_A_force.open("A_force_history.txt",std::ofstream::out | ofstream::trunc);
    P_fit_hist.open("P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);             //overall best fitness per stat run
    A_fit_hist.open("A_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);
    SR.open("P_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);                    //best fitness per generation for all SR
    SR_A.open("A_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);
    P_fit.open("stat_ave_best_P_fitness.txt", ofstream::out | ofstream::trunc);
    A_fit.open("stat_ave_best_A_fitness.txt", ofstream::out | ofstream::trunc);
    
    ofstream AIC_goal,AIC_startx,AIC_startxdot,AIC_displace;
    AIC_goal.open("ANT_goal_history_bestpergen.txt", std::ofstream::out | ofstream::trunc);
    AIC_startx.open("ANT_startx_history_bestpergen.txt", std::ofstream::out | ofstream::trunc);
    AIC_startxdot.open("ANT_startxdot_history_bestpergen.txt", std::ofstream::out | ofstream::trunc);
    AIC_displace.open("ANT_displace_history_bestpergen.txt", std::ofstream::out | ofstream::trunc);
    
    ofstream x_med,xdot_med,xdd_med, med_P_force,med_A_force, SR_med, SR_A_med;
    x_med.open("med_x_history.txt", std::ofstream::out | ofstream::trunc);
    xdot_med.open("med_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    xdd_med.open("med_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    med_P_force.open("med_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    med_A_force.open("med_A_force_history.txt",std::ofstream::out | ofstream::trunc);
    SR_med.open("P_med_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);       //best fitness per generation
    SR_A_med.open("A_med_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);
    
    // TEST //
    ofstream fout,best_tx,best_txdot,best_txdd, best_Pt_force,best_At_force,SR_test_best, test_P_fit_hist,fifty_history,fifty_var_test_x;
    best_tx.open("test_x_history.txt", std::ofstream::out | ofstream::trunc);
    best_txdot.open("test_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    best_txdd.open("test_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    best_Pt_force.open("test_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    best_At_force.open("test_A_force_history.txt",std::ofstream::out | ofstream::trunc);
    test_P_fit_hist.open("test_P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);               //best fitness per policy of single gen
    SR_test_best.open("test_P_best_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);           //best fitness out of all policies
    fifty_history.open("fiftysets_init.txt", ofstream::out | ofstream::trunc);
    fifty_var_test_x.open("best_50_var_x_history.txt",ofstream::out | ofstream::trunc);
    
    ofstream med_tx,med_txdot,med_txdd, med_Pt_force,med_At_force, SR_test_med;
    med_tx.open("med_test_x_history.txt", std::ofstream::out | ofstream::trunc);
    med_txdot.open("med_test_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    med_txdd.open("med_test_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    med_Pt_force.open("med_test_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    med_At_force.open("med_test_A_force_history.txt",std::ofstream::out | ofstream::trunc);
    SR_test_med.open("test_P_med_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);
    
    ofstream nsensor, nactuator, nposition, nvelocity, tstep_sensor, tstep_actuator,tstep_position, tstep_velocity;
    nsensor.open("ave_sensor_noise.txt", ofstream::out | ofstream::trunc);
    nactuator.open("ave_actuator_noise.txt", ofstream::out | ofstream::trunc);
    nposition.open("ave_position_noise.txt", ofstream::out | ofstream::trunc);
    nvelocity.open("ave_velocity_noise.txt", ofstream::out | ofstream::trunc);
    tstep_sensor.open("tstep_sensor.txt", ofstream::out | ofstream::trunc);
    tstep_actuator.open("tstep_actuator.txt", ofstream::out | ofstream::trunc);
    tstep_position.open("tstep_position.txt", ofstream::out | ofstream::trunc);
    tstep_velocity.open("tstep_velocity.txt", ofstream::out | ofstream::trunc);
}


#endif /* Parameters_hpp */
