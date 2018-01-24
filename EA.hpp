//
//  EA.hpp
//  Pro-Ant_EA_project
//
//  Created by Sierra Gonzales on 8/21/17.
//  Copyright © 2017 Pro-Ant_EA_project. All rights reserved.
//

#ifndef EA_hpp
#define EA_hpp

#include <iostream>
#include <cstdlib>
#include <vector>
#include <cmath>
#include <time.h>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <cassert>
#include <ctime>
#include <random>
#include "LY_NN.h"
#include <deque>

using namespace std;



class EA
{
    friend class Parameters;
    friend class Simulator;
    friend class Policy;
    friend class neural_network;
    
    
protected:
    
    
public:
    Parameters* pP;
    
    vector<Policy> pro_pol;
    vector<Policy> ant_pol;
    vector<Policy> test_pro_pol;
    vector<Policy> test_ant_pol;
    vector<double> best_P_fitness;
    vector<double> best_A_fitness;
    
    void Build_Population();
    void Run_Simulation();
    void Run_Test_Simulation();
    void Evaluate();
    int P_Binary_Select();
    int A_Binary_Select();
    void Downselect();
    void Mutation(Policy &M, Policy &N);
    void Repopulate();
    struct Less_Than_Policy_Fitness;
    struct Greater_Than_Policy_Fitness;
    void Sort_Policies_By_Fitness();
    void Sort_Test_Policies_By_Fitness();
    void EA_Process();
    void Run_Program();
    void Graph();
    void Graph_med();
    void Graph_test();
    double sum_P;
    double sum_A;
    double ave_P;
    double ave_A;
    int place;
    int num_loops;
    void init_fit();
    void determine_num_loops();
    
    // Fitness//
    void ave_fit();
    void update_best_fit();
    void update_best_test_fit();
    void full_leniency_ave_fit();
    
    
    deque<Policy> temporary_best_pro_policies(deque<Policy> pro_deq);
    deque<Policy> temporary_best_ant_policies(deque<Policy> ant_deq);
    
private:
};


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////////////
//Builds population of policies
void EA::Build_Population() {
    neural_network PNN;
    PNN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs);
    pP->num_weights = PNN.intended_size;
    //cout << pP->num_weights << endl;
    pro_pol.clear();
    ant_pol.clear();
    for (int i=0; i<pP->num_pol; i++) {
        Policy proP;
        Policy antP;
        pro_pol.push_back(proP);    //population of primary policies
        ant_pol.push_back(antP);
        pro_pol.at(i).age = 0;
        ant_pol.at(i).age = 0;
        pro_pol.at(i).P_fitness = 0;
        ant_pol.at(i).A_fitness = 0;
        //cout << "Policy" << "\t" << i << "\t" << "weights" << "\t";
        //for that policy have a vector of weights
        for (int w=0; w < pP->num_weights; w++){
            //pick random # between 0 and 1 and put into that vector
            double P_r = -1 + (2)*((double)rand()/RAND_MAX);
            double A_r = -1 + (2)*((double)rand()/RAND_MAX);
            pro_pol.at(i).P_weights.push_back(P_r);
            ant_pol.at(i).A_weights.push_back(A_r);
            
            assert(-1<=P_r && 1>=P_r);
            assert(-1<=A_r && 1>=A_r);
        }
        if (pP->rand_antagonist==true) { //if statement not necessary
            double A_IC_goal = double(rand() % 6);
            double A_IC_startx = 5 + double(rand() % 25);
            double A_IC_startxdot = double(rand() % 5);
            ant_pol.at(i).A_ICs.push_back(A_IC_goal); //goal_x
            ant_pol.at(i).A_ICs.push_back(A_IC_startx); //start_x
            ant_pol.at(i).A_ICs.push_back(A_IC_startxdot); //start_x_dot
        }
        
    }
    assert(pro_pol.size() == pP->num_pol); // check to make sure that the policy sizes are the same
    assert(ant_pol.size() == pP->num_pol);
}

void EA::init_fit(){
    for (int i=0; i<pP->num_pol; i++) {
        pro_pol.at(i).P_fitness = 0; //changed from -1 11/15
        ant_pol.at(i).A_fitness = 0;
        pro_pol.at(i).P_fit_swap = 0;
        ant_pol.at(i).A_fit_swap = 0;
    }
}

void EA::determine_num_loops(){
    if (pP->multi_var==true){
        num_loops=50;
        pP->fifty_var();    //initialize 50x3 variables
    }
    else {
        num_loops = 1;
    }
}

void EA::full_leniency_ave_fit(){
    if (pP->A_f_max_bound != 0){
        for (int b=0; b<pP->num_pol; b++){
            pro_pol.at(b).P_fitness = pro_pol.at(b).P_fitness/pP->num_pol;
            ant_pol.at(b).A_fitness = ant_pol.at(b).A_fitness/pP->num_pol;
            //cout << "pro\t" << pro_pol.at(b).P_fitness << endl;
            //cout << "ant\t" << ant_pol.at(b).A_fitness << endl;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
//Puts each policy into the simulation
void EA::Run_Simulation() {
    random_shuffle ( pro_pol.begin(), pro_pol.end() );
    random_shuffle ( ant_pol.begin(), ant_pol.end() );
    fstream rand_start, test_fit;
    rand_start.open("random_starting_variables.txt", fstream::app);
    test_fit.open("stat_Ptest_fitness.txt", fstream::app);
    if (pP->rand_start_gen == true){
        pP->random_variables();
    }
    determine_num_loops();
    
    
    
    //LOGGING START POSITIONS
    rand_start << pP->m << "\t" << pP->b << "\t" << pP->k << "\t" << pP->mu << "\t" << pP->start_x << "\t" << pP->goal_x << "\t" << pP->start_x_dot << endl;
    
    init_fit(); //P and A fitness and fitswap set to zero
    
    for (int i=0; i<pP->num_pol; i++) {
        
        for (int k=0; k<num_loops; k++) {
            if (pP->multi_var==true){
                pP->goal_x = pP->fifty_inits.at(k).at(0);       //goal from vector
                //cout << pP->goal_x << endl;
                pP->start_x = pP->fifty_inits.at(k).at(1);      //start x from vector
                pP->start_x_dot = pP->fifty_inits.at(k).at(2);  //start xdot from vector
            }
            
            //First we insert a policy into the simulator then we can take the objective data for that policy and store it in our data architecture
            Simulator S;
            S.pP = this->pP;
            Policy* pPo;
            Policy* aPo;
            pPo = & pro_pol.at(i);
            if (pP->A_f_max_bound !=0){ //coevolution - each policy against each other //should only run if there is an Antagonist
                for (int z= 0; z<pP->num_pol; z++){
                    aPo = & ant_pol.at(z);
                    S.Simulate(pPo, aPo);
                    
                    pro_pol.at(i).P_fitness += pro_pol.at(i).P_fit_swap;
                    ant_pol.at(z).A_fitness += ant_pol.at(z).A_fit_swap;
                    
                }
            }
            else{
                aPo = & ant_pol.at(i);
                S.Simulate(pPo, aPo);
                
                if (pro_pol.at(i).P_fitness < pro_pol.at(i).P_fit_swap) { //swapped direction 11/15 at 9:50
                    pro_pol.at(i).P_fitness = pro_pol.at(i).P_fit_swap;
                }
                if (ant_pol.at(i).A_fitness < ant_pol.at(i).A_fit_swap) {
                    ant_pol.at(i).A_fitness = ant_pol.at(i).A_fit_swap;
                }
                assert(pro_pol.at(i).P_fitness>=0);
                //assert(ant_pol.at(i).A_fitness>=0 && ant_pol.at(i).A_fitness<10000000);
                assert(ant_pol.at(i).A_fitness>=0);
            }
        }
        
        
        if (pP->multi_var==true) { //QUESTION: what was I thinking and how does this even work?
            pro_pol.at(i).P_fitness = pro_pol.at(i).P_fit_swap;
        }
    }
    
    full_leniency_ave_fit(); //if there is comparison between all policies, then the fitness of one policy against all policies is summed and divided by the num_pol
    
    
}

//////////////////////////////////////////////////////////////////////////////
//Puts each policy into the simulation
void EA::Run_Test_Simulation() {
    random_shuffle ( test_pro_pol.begin(), test_pro_pol.end() );
    random_shuffle ( test_ant_pol.begin(), test_ant_pol.end() );
    fstream test_fit;
    test_fit.open("stat_Ptest_fitness.txt", fstream::app);

    init_fit(); //P and A fitness and fitswap set to zero
    //assert(pP->A_f_max_bound ==0 && pP->A_f_min_bound ==0);
    
    for (int i=0; i<pP->num_pol; i++) {
        //First we insert a policy into the simulator then we can take the objective data for that policy and store it in our data architecture
        Simulator S;
        S.pP = this->pP;
        Policy* pPo;
        Policy* aPo;
        pPo = & test_pro_pol.at(i);
        
        aPo = & test_ant_pol.at(i);
        S.Simulate(pPo, aPo);
                
        if (test_pro_pol.at(i).P_fitness < test_pro_pol.at(i).P_fit_swap) { //swapped direction 11/15 at 9:50
            test_pro_pol.at(i).P_fitness = test_pro_pol.at(i).P_fit_swap;
        }
        if (test_ant_pol.at(i).A_fitness < test_ant_pol.at(i).A_fit_swap) {
            test_ant_pol.at(i).A_fitness = test_ant_pol.at(i).A_fit_swap;
        }
        assert(test_pro_pol.at(i).P_fitness>=0);
        assert(test_ant_pol.at(i).A_fitness>=0 && test_ant_pol.at(i).A_fitness<10000000);
                
        
    }
    
    fstream nsensor, nactuator;
    nsensor.open("ave_sensor_noise.txt", fstream::app);
    nactuator.open("ave_actuator_noise.txt", fstream::app);
    nsensor << endl;
    nactuator << endl;
    
    
}


//////////////////////////////////////////////////////////////////////////////
//Evaluates each policies fitness scores for each objective
void EA::Evaluate() {
    //might not need
    for (int i=0; i< pP->num_pol; i++) {
        
    }
}


//////////////////////////////////////////////////////////////////////////////
//Randomly selects two individuals and decides which one will die based on their fitness
int EA::P_Binary_Select() {
    int loser;
    int index_1 = rand() % pro_pol.size();
    int index_2 = rand() % pro_pol.size();
    while (index_1 == index_2) {
        index_2 = rand() % pro_pol.size();
    }
    
    //winner is one with lower fitness
    if(pro_pol.at(index_1).P_fitness < pro_pol.at(index_2).P_fitness) {
        loser = index_2;
    }
    else {
        loser = index_1;
    }
    return loser;
}

int EA::A_Binary_Select() {
    int a_loser;
    int index_3 = rand() % ant_pol.size();
    int index_4 = rand() % ant_pol.size();
    while (index_3 == index_4) {
        index_4 = rand() % ant_pol.size();
    }
    
    //winner is one with higher fitness
    if(ant_pol.at(index_3).A_fitness > ant_pol.at(index_4).A_fitness) {
        a_loser = index_4;
    }
    else {
        a_loser = index_3;
    }
    return a_loser;
}



//-------------------------------------------------------------------------
//Policies are compared to determine the optimal policies for a given generation
void EA::Downselect() {
    int pro_kill;
    int ant_kill;
    for(int k=0; k<pP->to_kill; k++) {
        pro_kill = P_Binary_Select();               //Protagonist
        pro_pol.erase(pro_pol.begin() + pro_kill);
        ant_kill = A_Binary_Select();               //Antagonist
        ant_pol.erase(ant_pol.begin() + ant_kill);
    }
    assert(pro_pol.size() == pP->to_kill);
    assert(ant_pol.size() == pP->to_kill);
    for(int i=0; i<pP->to_kill; i++) {
        pro_pol.at(i).age += 1;
        ant_pol.at(i).age += 1;
    }
}


//-------------------------------------------------------------------------
//Mutates the copies of the winning individuals
void EA::Mutation(Policy &M, Policy &N) {
    //This is where the policy is slightly mutated
    for (int x = 0; x < pP->num_weights; x++) {
        double random = ((double)rand()/RAND_MAX);
        double random2 = ((double)rand()/RAND_MAX);
        
        // PROTAGONIST //
        if (random <= pP->mutation_rate) {
            double R1 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            double R2 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            M.P_weights.at(x) = M.P_weights.at(x) + (R1-R2);
            if (M.P_weights.at(x) < -1) {
                M.P_weights.at(x) = -1;
            }
            if (M.P_weights.at(x) > 1) {
                M.P_weights.at(x) = 1;
            }
        }
        assert(M.P_weights.at(x)<=1 && M.P_weights.at(x)>=-1);
        
        // ANTAGONIST //
        if (pP->A_f_max_bound != 0){ //If there is an Antagonist... then mutate
            if (random2 <= pP->mutation_rate) {
                double R3 = ((double)rand()/RAND_MAX) * pP->mutate_range;
                double R4 = ((double)rand()/RAND_MAX) * pP->mutate_range;
                
                N.A_weights.at(x) = N.A_weights.at(x) + (R3-R4);
                if (N.A_weights.at(x) < -1) {
                    N.A_weights.at(x) = -1;
                }
                if (N.A_weights.at(x) > 1) {
                    N.A_weights.at(x) = 1;
                }
                assert(N.A_weights.at(x)<=1 && N.A_weights.at(x)>=-1);
            }
        }
    }
    if (pP->rand_antagonist==true){
        double random2 = ((double)rand()/RAND_MAX);
        if (random2 <= pP->mutation_rate) {
            double R3 = double(rand() % 1);
            double R4 = double(rand() % 1);
            //GOAL_X SET AND BOUNDARY CHECK
            N.A_ICs.at(0) = pP->goal_x + R3 - R4;
            if (N.A_ICs.at(0)>pP->goal_x_upper_bound){
                N.A_ICs.at(0) =pP->goal_x_upper_bound;
            }
            if (N.A_ICs.at(0)<pP->goal_x_lower_bound){
                N.A_ICs.at(0) =pP->goal_x_lower_bound;
            }
            //START_X SET AND BOUNDARY CHECK
            N.A_ICs.at(1) = pP->start_x + R3 - R4;
            if (N.A_ICs.at(1)>pP->start_x_upper_bound){
                N.A_ICs.at(1) =pP->start_x_upper_bound;
            }
            if (N.A_ICs.at(1)<pP->start_x_lower_bound){
                N.A_ICs.at(1) =pP->start_x_lower_bound;
            }
            //START_X_DOT SET AND BOUNDARY CHECK
            N.A_ICs.at(2)= pP->start_x_dot + R3 - R4;
            if (N.A_ICs.at(2)>pP->start_x_dot_upper_bound){
                N.A_ICs.at(2) =pP->start_x_dot_upper_bound;
            }
            if (N.A_ICs.at(2)<pP->start_x_dot_lower_bound){
                N.A_ICs.at(2) =pP->start_x_dot_lower_bound;
            }
        }
    }
}


//-------------------------------------------------------------------------
//The population is repopulated based on small mutations of the remaining policies
void EA::Repopulate() {
    int to_replicate = pP->to_kill;
    for (int rep=0; rep<to_replicate; rep++) {
        Policy M;
        Policy N;
        int spot = rand() % pro_pol.size();
        int spot2 = rand() % ant_pol.size();
        M = pro_pol.at(spot);
        N = ant_pol.at(spot2);
        
        Mutation(M, N);
        pro_pol.push_back(M);
        ant_pol.push_back(N);
        pro_pol.at(pro_pol.size()-1).age = 0; //how long it has survived
        ant_pol.at(ant_pol.size()-1).age = 0;
    }
    assert(pro_pol.size() == pP->num_pol);
    assert(ant_pol.size() == pP->num_pol);
}


//-------------------------------------------------------------------------
//Runs the entire EA loop process
void EA::EA_Process() {
    Run_Simulation();
    Evaluate();
    Sort_Policies_By_Fitness();
    Downselect();
    Repopulate();
}


//-------------------------------------------------------------------------
//Sorts the population based on their fitness from lowest to highest
struct EA::Less_Than_Policy_Fitness {
    inline bool operator() (const Policy& struct1, const Policy& struct2) {
        return (struct1.P_fitness < struct2.P_fitness);
    }
};

struct EA::Greater_Than_Policy_Fitness {
    inline bool operator() (const Policy& struct3, const Policy& struct4) {
        return (struct3.A_fitness > struct4.A_fitness);
    }
};

//-------------------------------------------------------------------------
//Sorts population
void EA::Sort_Policies_By_Fitness() {
    for (int i=0; i<pP->num_pol; i++) {
        sort(pro_pol.begin(), pro_pol.end(), Less_Than_Policy_Fitness());
        sort(ant_pol.begin(), ant_pol.end(), Greater_Than_Policy_Fitness());
    }
    update_best_fit();
}

//-------------------------------------------------------------------------
//Sorts population
void EA::Sort_Test_Policies_By_Fitness() {
    for (int i=0; i<pP->num_pol; i++) {
        sort(test_pro_pol.begin(), test_pro_pol.end(), Less_Than_Policy_Fitness());
        sort(test_ant_pol.begin(), test_ant_pol.end(), Greater_Than_Policy_Fitness());
    }
    
}

//-------------------------------------------------------------------------
void EA::update_best_fit(){
    best_P_fitness.push_back(pro_pol.at(0).P_fitness);      // best fitness per generation
    best_A_fitness.push_back(ant_pol.at(0).A_fitness);
    
    
}

deque<Policy> EA::temporary_best_pro_policies(deque<Policy> pro_deq){
    pro_deq.push_front(pro_pol.at(0)); //saves best per gen of last 5 generations
    //cout << pro_deq.at(0).P_fitness << "\t";
    pro_deq.resize(5);
    
    assert(pro_deq.size()==5);
    return pro_deq;
}
deque<Policy> EA::temporary_best_ant_policies(deque<Policy> ant_deq){
    
    ant_deq.push_front(ant_pol.at(0)); // this push front then resize, gives you the last 5 like this: 100 99 98 97 96 instead of flipped
    ant_deq.resize(5);
    
    assert(ant_deq.size()==5);
    return ant_deq;
}

//-------------------------------------------------------------------------
void EA::update_best_test_fit(){
    ofstream P_testperfive_fit;
    P_testperfive_fit.open("stat_P_testperfive_fit.txt", fstream::app);
    P_testperfive_fit << test_pro_pol.at(0).P_fitness << "\t";      // best fitness per generation
    //cout << test_pro_pol.at(0).P_fitness << "\t";
}

void EA::ave_fit(){
    sum_P=0;
    sum_A=0;
    ofstream P_fit, A_fit;
    P_fit.open("stat_ave_best_P_fitness.txt", fstream::app);
    A_fit.open("stat_ave_best_A_fitness.txt", fstream::app);
    assert(best_P_fitness.size() == pP->gen_max);
    for (int f = 0; f < best_P_fitness.size(); f++) { //help 11/20
        //sum_P += pro_pol.at(f).P_fitness;
        sum_P += best_P_fitness.at(f);
        sum_A += best_A_fitness.at(f);
    }
    ave_P = sum_P/best_P_fitness.size();
    ave_A = sum_A/best_A_fitness.size();
    P_fit << ave_P << endl;
    A_fit << ave_A << endl;
}


//-------------------------------------------------------------------------
void EA::Graph(){
    
    // BEST //
    ofstream x_best,xdot_best,xdd_best, best_P_force,best_A_force, P_fit_hist, A_fit_hist, SR, SR_A;
    //NOTE
    //pro_pol.at(0) is best
    int best = 0;
    //pro_pol.size()/2 is median
    int median = pro_pol.size()/2;
    
    if (pP->best_vs_median==true){
        place = best; //graph the best
    }
    else {
        place = median; //graph the median
    }
    
    x_best.open("x_history.txt", fstream::app);
    xdot_best.open("x_dot_history.txt",fstream::app);
    xdd_best.open("x_dd_history.txt",fstream::app);
    
    best_P_force.open("P_force_history.txt",fstream::app);
    best_A_force.open("A_force_history.txt",fstream::app);
    
    //fout << "vector spot" << "\t" << "x" << "\t" << "x_dot" << "\t" << "x_dd" << endl;
    for (int f =0; f < pP->total_time; f++){
        x_best << pro_pol.at(best).x_history.at(f) << "\t";
        xdot_best << pro_pol.at(best).x_dot_history.at(f) << "\t";
        xdd_best << pro_pol.at(best).x_dd_history.at(f) << "\t";
        
        best_P_force << pro_pol.at(best).P_force_history.at(f) << "\t";
        best_A_force << ant_pol.at(best).A_force_history.at(f) << "\t";
    }
    
    x_best << endl;
    xdot_best << endl;
    xdd_best << endl;
    best_P_force << endl;
    best_A_force << endl;
    
    x_best.close();
    xdot_best.close();
    xdd_best.close();
    
    best_P_force.close();
    best_A_force.close();
    
    P_fit_hist.open("P_best_fitness_history.txt",fstream::app);     //best fitness out of all generations
    A_fit_hist.open("A_best_fitness_history.txt",fstream::app);     //best fitness out of all generations
    assert(best_P_fitness.size() == pP->gen_max);
    P_fit_hist << best_P_fitness.at(0) << endl;
    A_fit_hist << best_A_fitness.at(0) << endl;
    
    SR.open("P_best_fitpergen_SR_history.txt", fstream::app);       //best fitness per generation
    SR_A.open("A_best_fitpergen_SR_history.txt", fstream::app);     //best fitness per generation
    for (int h =0; h < pP->gen_max; h++){
        SR << best_P_fitness.at(h) << "\t";
        SR_A << best_A_fitness.at(h) << "\t";
    }
    SR << endl;
    SR_A << endl;
    
    // MEDIAN //
    ofstream x_med,xdot_med,xdd_med, med_P_force,med_A_force, SR_med, SR_A_med;
    x_med.open("med_x_history.txt", fstream::app);
    xdot_med.open("med_x_dot_history.txt",fstream::app);
    xdd_med.open("med_x_dd_history.txt",fstream::app);
    
    med_P_force.open("med_P_force_history.txt",fstream::app);
    med_A_force.open("med_A_force_history.txt",fstream::app);
    
    x_med << endl;
    xdot_med  << endl;
    xdd_med  << endl;
    med_P_force << endl;
    med_A_force << endl;
    
    
    
    SR_med.open("P_med_fitpergen_SR_history.txt", fstream::app);       //best fitness per generation
    SR_A_med.open("A_med_fitpergen_SR_history.txt", fstream::app);     //best fitness per generation
    SR_med << endl;
    SR_A_med << endl;
}

void EA::Graph_med(){
    // MEDIAN //
    ofstream med_x,med_xdot,med_xdd, med_P_force,med_A_force, SR_med, SR_A_med;
    
    med_x.open("med_x_history.txt", fstream::app);
    med_xdot.open("med_x_dot_history.txt",fstream::app);
    med_xdd.open("med_x_dd_history.txt",fstream::app);
    
    med_P_force.open("med_P_force_history.txt",fstream::app);
    med_A_force.open("med_A_force_history.txt",fstream::app);
    
    SR_med.open("P_med_fitpergen_SR_history.txt", fstream::app);
    SR_A_med.open("A_med_fitpergen_SR_history.txt", fstream::app);     //best fitness per generation
    
    for (int f =0; f < pP->total_time; f++){
        med_x << pro_pol.at(pro_pol.size()/2).x_history.at(f) << "\t";
        med_xdot << pro_pol.at(pro_pol.size()/2).x_dot_history.at(f) << "\t";
        med_xdd << pro_pol.at(pro_pol.size()/2).x_dd_history.at(f) << "\t";
        
        med_P_force << pro_pol.at(pro_pol.size()/2).P_force_history.at(f) << "\t";
        med_A_force << ant_pol.at(pro_pol.size()/2).A_force_history.at(f) << "\t";
        
    }
    
    SR_med << pro_pol.at(pro_pol.size()/2).P_fitness << "\t";
    SR_A_med << ant_pol.at(ant_pol.size()/2).A_fitness << "\t";
    
    
    med_x.close();
    med_xdot.close();
    med_xdd.close();
    
    med_P_force.close();
    med_A_force.close();
}


void EA::Graph_test(){
    // BEST //
    ofstream best_tx,best_txdot,best_txdd, best_Pt_force,best_At_force;
    
    best_tx.open("test_x_history.txt", fstream::app);
    best_txdot.open("test_x_dot_history.txt",fstream::app);
    best_txdd.open("test_x_dd_history.txt",fstream::app);
    
    best_Pt_force.open("test_P_force_history.txt",fstream::app);
    best_At_force.open("test_A_force_history.txt",fstream::app);
    
    ofstream SR_test_best, test_P_fit_hist;
    test_P_fit_hist.open("test_P_best_fitness_history.txt",fstream::app);       //fitness per policy
    SR_test_best.open("test_P_best_fitpergen_SR_history.txt", fstream::app);         //best fitness out of all policies
    
    for (int f =0; f < pP->total_time; f++){
        best_tx << pro_pol.at(0).x_history.at(f) << "\t";
        best_txdot << pro_pol.at(0).x_dot_history.at(f) << "\t";
        best_txdd << pro_pol.at(0).x_dd_history.at(f) << "\t";
        
        best_Pt_force << pro_pol.at(0).P_force_history.at(f) << "\t";
        best_At_force << ant_pol.at(0).A_force_history.at(f) << "\t";

    }
    
    best_tx << endl;
    best_txdot << endl;
    best_txdd << endl;
    
    best_Pt_force << endl;
    best_At_force << endl;
    
    best_tx.close();
    best_txdot.close();
    best_txdd.close();
    
    best_Pt_force.close();
    best_At_force.close();
 
    
    for (int h =0; h < pP->num_pol; h++){
        test_P_fit_hist << pro_pol.at(h).P_fitness << "\t";
        
    }
    test_P_fit_hist << endl;
    SR_test_best << best_P_fitness.back() << endl;
    
    ofstream med_tx,med_txdot,med_txdd, med_Pt_force,med_At_force, SR_test_med;
    
    med_tx.open("med_test_x_history.txt", fstream::app);
    med_txdot.open("med_test_x_dot_history.txt",fstream::app);
    med_txdd.open("med_test_x_dd_history.txt",fstream::app);
    med_Pt_force.open("med_test_P_force_history.txt",fstream::app);
    med_At_force.open("med_test_A_force_history.txt",fstream::app);
    SR_test_med.open("test_P_med_fitpergen_SR_history.txt", fstream::app);
    
    for (int f =0; f < pP->total_time; f++){
        med_tx << pro_pol.at(pro_pol.size()/2).x_history.at(f) << "\t";
        med_txdot << pro_pol.at(pro_pol.size()/2).x_dot_history.at(f) << "\t";
        med_txdd << pro_pol.at(pro_pol.size()/2).x_dd_history.at(f) << "\t";
        
        med_Pt_force << pro_pol.at(pro_pol.size()/2).P_force_history.at(f) << "\t";
        med_At_force << ant_pol.at(pro_pol.size()/2).A_force_history.at(f) << "\t";
        
    }
    
    SR_test_med << pro_pol.at(pro_pol.size()/2).P_fitness << "\t";
    
    
    med_tx.close();
    med_txdot.close();
    med_txdd.close();
    
    med_Pt_force.close();
    med_At_force.close();
    
    
    
    
    // ALL NOISE //
    ofstream tstep_position, tstep_velocity, tstep_sensor, tstep_actuator,nsensor, nactuator, nposition, nvelocity;
    tstep_sensor.open("tstep_sensor.txt", ofstream::app);
    tstep_actuator.open("tstep_actuator.txt", ofstream::app);
    tstep_position.open("tstep_position.txt", ofstream::app);
    tstep_velocity.open("tstep_velocity.txt", ofstream::app);
    
    nsensor.open("ave_sensor_noise.txt", ofstream::app);
    nactuator.open("ave_actuator_noise.txt", ofstream::app);
    nposition.open("ave_position_noise.txt", ofstream::app);
    nvelocity.open("ave_velocity_noise.txt", ofstream::app);
    for (int h =0; h < pP->num_pol; h++){
        assert(pP->total_time == pro_pol.at(h).position_noise_tstep_history.size());
        for (int j =0; j < pP->total_time; j++){
            // TIMESTEP NOISE HISTORY //
            tstep_position << pro_pol.at(h).position_noise_tstep_history.at(j) << "\t";
            //cout << pro_pol.at(h).position_noise_tstep_history.at(j) << "\t";
            tstep_velocity << pro_pol.at(h).velocity_noise_tstep_history.at(j) << "\t";
            tstep_sensor << pro_pol.at(h).sensor_noise_tstep_history.at(j) << "\t";
            tstep_actuator << pro_pol.at(h).actuator_noise_tstep_history.at(j) << "\t";
        }
        tstep_position << endl;
        tstep_velocity << endl;
        tstep_sensor << endl;
        tstep_actuator << endl;
        assert(pro_pol.at(h).ave_sensor_noise_history.size() == 1);
        nsensor << pro_pol.at(h).ave_sensor_noise_history.at(0) << "\t";
        nactuator << pro_pol.at(h).ave_actuator_noise_history.at(0) << "\t";
        nposition << pro_pol.at(h).ave_position_noise_history.at(0) << "\t";
        nvelocity << pro_pol.at(h).ave_velocity_noise_history.at(0) << "\t";
    }
    nsensor << endl;
    nactuator << endl;
    nposition << endl;
    nvelocity << endl;
    
    
    
}



//-------------------------------------------------------------------------
//Runs the entire program
void EA::Run_Program() {
    ofstream rand_start, P_fit, A_fit,P_testperfive_fit;
    
    rand_start.open("random_starting_variables.txt", ofstream::out | ofstream::trunc);
    P_fit.open("stat_ave_best_P_fitness.txt", fstream::app);
    A_fit.open("stat_ave_best_A_fitness.txt", fstream::app);
    P_testperfive_fit.open("stat_P_testperfive_fit.txt", fstream::app);
    
    Build_Population();
    best_P_fitness.clear();
    best_A_fitness.clear();
    
    deque<Policy> pro_deq;
    deque<Policy> ant_deq;
    for (int i=0; i<5; i++){
        //pro_deq[i]=0;
    }
    //assert(pro_deq.size()==5);
    
    for (int gen=0; gen<pP->gen_max; gen++) {
        if (gen %5 ==0){
            if (pP->rand_start_5gen==true){
                pP->random_variables(); //different every stat run
            }
            
        }
        if (gen %10 ==0) {
            //cout << "GENERATION \t" << gen << endl;
        }
        if (gen %5 == 0){
            //cout << "GENERATION \t" << gen << endl;
            if (pP->testperfive == true){
                //run test
                //copy the policies of last gen?
                test_pro_pol = pro_pol;
                test_ant_pol = ant_pol;
                pP->te_A =true;
                pP->test();//change parameters
                Run_Test_Simulation();
                Sort_Test_Policies_By_Fitness();//sort then push best fit into a file
                update_best_test_fit();
                if (pP->two_A==true) {
                    pP->tr_2=true;
                    //cout<< "train 2 chosen" << endl;
                }
                else if (pP->three_A==true){
                     pP->tr_3=true;
                    //cout<< "train 3 chosen" << endl;
                }
                pP->train(); //change parameters back
                assert(pP->actuator_NOISE==false && pP->sensor_NOISE ==false);
            }
        }
        
        if (gen < pP->gen_max-1) {
            
            if (gen>pP->ant_intro){
                if (pP->late_antagonist==true) {
                    pP->A_f_min_bound = -1;
                    pP->A_f_max_bound = 1;
                }
            }
            EA_Process();
            Graph_med();
            pro_deq = temporary_best_pro_policies(pro_deq);
            ant_deq = temporary_best_ant_policies(ant_deq);

        }
        else {
            
            Run_Simulation();
            Evaluate();
            Sort_Policies_By_Fitness();
            pro_deq = temporary_best_pro_policies(pro_deq);
            ant_deq = temporary_best_ant_policies(ant_deq);

            cout << "BEST POLICY PRO-FITNESS" << "\t" << pro_pol.at(0).P_fitness << endl;
            //P_fit << pro_pol.at(0).P_fitness << endl;
            Graph_med();
            
            
        }
    }
    ave_fit();
    Graph();
    for (int i=0; i<5; i++){
        cout << pro_deq[i].P_fitness << "\t";
    }
    cout << endl;
    for (int i=0; i<5; i++){
        cout << ant_deq[i].A_fitness << "\t";
    }
    cout << endl;
    
    P_testperfive_fit << endl;
    //cout << endl;
    P_fit.close();
    A_fit.close();

    
    rand_start.close();
}


#endif /* EA_hpp */
