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
    vector<double> best_P_fitness;
    vector<double> best_A_fitness;

    void Build_Population();
    void Run_Simulation();
    void Evaluate();
    int P_Binary_Select();
    int A_Binary_Select();
    void Downselect();
    void Mutation(Policy &M, Policy &N);
    void Repopulate();
    struct Less_Than_Policy_Fitness;
    struct Greater_Than_Policy_Fitness;
    void Sort_Policies_By_Fitness();
    void EA_Process();
    void Run_Program();
    
    void Graph_test();
    double sum;
    double ave;
    int place;
    
    
    //TEST
    void Run_Test_Program();
    void Run_Test_Simulation();
    void Sort_Test_Policies_By_Fitness();
    void test_init_fit();
    vector<Policy> test_pro_pol;
    vector<Policy> test_ant_pol;
    
    //Fitness
     void update_best_fit();
    void ave_fit();
    double sum_P;
    double sum_A;
    double ave_P;
    double ave_A;
    
    //Graph
    void Graph();
    void Graph_med();
    
    
private:
};


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////////////
//Builds population of policies
void EA::Build_Population() {
    neural_network PNN, ANN;
    PNN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs);
    pP->num_weights = PNN.intended_size;
    
    ANN.setup(pP->A_num_inputs,pP->num_nodes,pP->num_outputs);
    pP->A_num_weights = ANN.intended_size;
    
    pro_pol.clear();
    ant_pol.clear();
    for (int i=0; i<pP->num_pol; i++) {
        assert(ant_pol.size()==i);
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
            pro_pol.at(i).P_weights.push_back(P_r);
            assert(-1<=P_r && 1>=P_r);
            
        }
        assert(pro_pol.at(i).P_weights.size() == pP->num_weights);
        
        for (int v=0; v< pP->A_num_weights; v++) {
            double A_r = -1 + (2)*((double)rand()/RAND_MAX);
            ant_pol.at(i).A_weights.push_back(A_r);
            assert(-1<=A_r && 1>=A_r);
        }
        assert(ant_pol.at(i).A_weights.size() == pP->A_num_weights);
        
        if (pP->rand_antagonist==true) { //if statement not necessary
            double A_IC_goal = 0 + double(rand() % 5);
            double A_IC_startx = 10 + double(rand() % 10);
            double A_IC_startxdot = 0 + double(rand() % 5);
            double A_IC_displace = -2 + double(rand() % 4);
            assert(A_IC_startx<20);
            ant_pol.at(i).A_ICs.push_back(A_IC_goal);           //goal_x
            ant_pol.at(i).A_ICs.push_back(A_IC_startx);         //start_x
            ant_pol.at(i).A_ICs.push_back(A_IC_startxdot);      //start_x_dot
            ant_pol.at(i).A_ICs.push_back(A_IC_displace);
            assert(ant_pol.at(i).A_ICs.size()==4);
        }
        
    }
    assert(pro_pol.size() == pP->num_pol); // check to make sure that the policy sizes are the same
    assert(ant_pol.size() == pP->num_pol);
}
//----------------------------------------------
void EA::test_init_fit(){
    for (int i=0; i<pP->num_pol; i++) {
        test_pro_pol.at(i).P_fitness = 0; //changed from -1 11/15
        test_ant_pol.at(i).A_fitness = 0;
        test_pro_pol.at(i).P_fit_swap = 0;
        test_ant_pol.at(i).A_fit_swap = 0;
    }
}
//////////////////////////////////////////////////////////////////////////////
void EA::Run_Test_Simulation() {
    random_shuffle ( test_pro_pol.begin(), test_pro_pol.end() );
    random_shuffle ( test_ant_pol.begin(), test_ant_pol.end() );
    fstream test_fit;
    test_fit.open("stat_Ptest_fitness.txt", fstream::app);
    
    assert(pP->tr_2==false || pP->tr_3==false || pP->tr_4==false || pP->tr_5==false);
    
    
    test_init_fit(); //P and A fitness and fitswap set to zero
    //assert(pP->A_f_max_bound ==0 && pP->A_f_min_bound ==0);
    
    if (pP->multi_var==true){
        pP->num_loops=50;
    }
    else {
        pP->num_loops = 1;
    }
    
    for (int i=0; i<pP->num_pol; i++) {
        //First we insert a policy into the simulator then we can take the objective data for that policy and store it in our data architecture
        //test_init_fit(); //P and A fitness and fitswap set to zero
        
        test_pro_pol.at(i).loop_x_history.clear();
        pP->fifty_fitness.clear();
        assert(test_pro_pol.at(i).loop_x_history.size()==0);
        
        for (int k=0; k<pP->num_loops; k++){
            test_pro_pol.at(i).P_fitness = 0; //changed from -1 11/15
            test_ant_pol.at(i).A_fitness = 0;
            test_pro_pol.at(i).P_fit_swap = 0;
            test_ant_pol.at(i).A_fit_swap = 0;
            if (pP->multi_var==true){
                assert(pP->num_loops==50);
                pP->goal_x = pP->fifty_inits.at(k).at(0);       //goal from vector
                //cout << pP->goal_x << endl;
                pP->start_x = pP->fifty_inits.at(k).at(1);      //start x from vector
                pP->start_x_dot = pP->fifty_inits.at(k).at(2);  //start xdot from vector
                pP->displace = pP->fifty_inits.at(k).at(3);     //dispace from vector
            }
            else{
                pP->start_x = pP->init_start_x;
                pP->start_x_dot = pP->init_start_x_dot;
                pP->goal_x = pP->init_goal_x;
                pP->displace = pP->init_displace;
            }
            
            Simulator S;
            S.pP = this->pP;
            Policy* pPo;
            Policy* aPo;
            pPo = & test_pro_pol.at(i);
            aPo = & test_ant_pol.at(i);
            S.Simulate(pPo, aPo);
            
            
            if (test_pro_pol.at(i).P_fitness < test_pro_pol.at(i).P_fit_swap) { //if > then make sure P_fitness is set to a very high number
                test_pro_pol.at(i).P_fitness = test_pro_pol.at(i).P_fit_swap;
            }
            if (test_ant_pol.at(i).A_fitness < test_ant_pol.at(i).A_fit_swap) {
                test_ant_pol.at(i).A_fitness = test_ant_pol.at(i).A_fit_swap;
            }
            else{
                cout << "whoops" << endl;
            }
            assert(test_pro_pol.at(i).P_fitness>=0);
            assert(test_ant_pol.at(i).A_fitness>=0 );
            
            if (pP->multi_var==true){
                pP->fifty_fitness.push_back(test_pro_pol.at(i).P_fitness);
                assert(pP->fifty_fitness.at(k)==test_pro_pol.at(i).P_fit_swap);
                assert(pP->fifty_fitness.size()==k+1);
                test_pro_pol.at(i).loop_x_history.push_back(pPo->x_history);
                assert(test_pro_pol.at(i).loop_x_history.at(k).size() == pP->total_time);
            }
            if (pP->fifty_fitness.size()==pP->num_loops){
                assert(k == pP->num_loops-1);
                for (int b=0; b<pP->num_loops; b++){
                    test_pro_pol.at(i).P_fitness += pP->fifty_fitness.at(b);
                }
            }

        }
        if (pP->multi_var==true) {
            assert(pP->num_loops == 50);
            assert(pP->fifty_fitness.size()==pP->num_loops);
            test_pro_pol.at(i).P_fitness = test_pro_pol.at(i).P_fitness/pP->num_loops;
            assert(test_pro_pol.at(i).loop_x_history.size() == pP->num_loops);
            
        }
        
        
    }
    
    fstream nsensor, nactuator;
    nsensor.open("ave_sensor_noise.txt", fstream::app);
    nactuator.open("ave_actuator_noise.txt", fstream::app);
    nsensor << endl;
    nactuator << endl;
    
    
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
        pP->random_start_end_variables();
    }
    if (pP->multi_var==true){
        pP->num_loops=50;
    }
    else {
        pP->num_loops = 1;
    }
    
    //LOGGING START POSITIONS
    rand_start << pP->m << "\t" << pP->b << "\t" << pP->k << "\t" << pP->mu << "\t" << pP->start_x << "\t" << pP->goal_x << "\t" << pP->start_x_dot << "\t" << pP->displace << endl;
    
    for (int i=0; i<pP->num_pol; i++) {
        pro_pol.at(i).P_fitness = 0; //changed from -1 11/15
        ant_pol.at(i).A_fitness = 0;
        pro_pol.at(i).P_fit_swap = 0;
        ant_pol.at(i).A_fit_swap = 0;
        for (int k=0; k<pP->num_loops; k++) {
            if (pP->multi_var==true){
                pP->goal_x = pP->fifty_inits.at(k).at(0);       //goal from vector
                pP->start_x = pP->fifty_inits.at(k).at(1);      //start x from vector
                pP->start_x_dot = pP->fifty_inits.at(k).at(2);  //start xdot from vector
                pP->displace = pP->fifty_inits.at(k).at(3);     //dispace from vector
            }
            else{
                pP->start_x = pP->init_start_x;
                pP->start_x_dot = pP->init_start_x_dot;
                pP->goal_x = pP->init_goal_x;
                pP->displace = pP->init_displace;
            }
            
            //First we insert a policy into the simulator then we can take the objective data for that policy and store it in our data architecture
            Simulator S;
            S.pP = this->pP;
            Policy* pPo;
            Policy* aPo;
            pPo = & pro_pol.at(i);
            if (pP->full_leniency==true){
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
                assert(ant_pol.at(i).A_fitness>=0 && ant_pol.at(i).A_fitness<10000000);
                
            }
            
        }
        if (pP->multi_var==true) {
            //test_fit << pro_pol.at(i).P_fit_swap << endl;
            pro_pol.at(i).P_fitness = pro_pol.at(i).P_fit_swap;
        }
    }
    
    
    fstream nsensor;
    fstream nactuator;
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
            if (M.P_weights.at(x)<-1) {
                M.P_weights.at(x) = -1;
            }
            if (M.P_weights.at(x)>1) {
                M.P_weights.at(x) = 1;
            }
        }
        assert(M.P_weights.at(x)<=1 && M.P_weights.at(x)>=-1);
        
        // ANTAGONIST //
        if (pP->tr_3==true){
            if (random2 <= pP->mutation_rate) {
                double R3 = ((double)rand()/RAND_MAX) * pP->mutate_range;
                double R4 = ((double)rand()/RAND_MAX) * pP->mutate_range;
                
                N.A_weights.at(x) = N.A_weights.at(x) + (R3-R4);
                if (N.A_weights.at(x)<-1) {
                    N.A_weights.at(x) = -1;
                }
                if (N.A_weights.at(x)>1) {
                    N.A_weights.at(x) = 1;
                }
                assert(N.A_weights.at(x)<=1 && N.A_weights.at(x)>=-1);
            }
        }
    }
    if (pP->rand_antagonist==true){
        for (int v=0; v < 4; v++){
            double random2 = ((double)rand()/RAND_MAX);
            if (v==0 && random2 <= pP->mutation_rate) {
                double R3 = ((double)rand()/RAND_MAX) * 1;
                double R4 = ((double)rand()/RAND_MAX) * 1;
                //cout << R3 << "\t" << R4 << endl;
                //GOAL_X SET AND BOUNDARY CHECK
                N.A_ICs.at(0) = pP->goal_x + R3 - R4;
                if (N.A_ICs.at(0)>pP->goal_x_upper_bound){
                    N.A_ICs.at(0) =pP->goal_x_upper_bound;
                }
                if (N.A_ICs.at(0)<pP->goal_x_lower_bound){
                    N.A_ICs.at(0) =pP->goal_x_lower_bound;
                }
            }
            else if (v==1 && random2 <= pP->mutation_rate){
                //START_X SET AND BOUNDARY CHECK
                double R5 = ((double)rand()/RAND_MAX) * 1;
                double R6 = ((double)rand()/RAND_MAX) * 1;
                N.A_ICs.at(1) = pP->start_x + R5 - R6;
                if (N.A_ICs.at(1)>pP->start_x_upper_bound){
                    N.A_ICs.at(1) =pP->start_x_upper_bound;
                }
                if (N.A_ICs.at(1)<10){
                    N.A_ICs.at(1) =10;
                }
                /*
                if (N.A_ICs.at(1)>pP->start_x_upper_bound){
                    N.A_ICs.at(1) =pP->start_x_upper_bound;
                }
                if (N.A_ICs.at(1)<pP->start_x_lower_bound){
                    N.A_ICs.at(1) =pP->start_x_lower_bound;
                }*/
            }
            else if (v==2 && random2 <= pP->mutation_rate){
                //START_X_DOT SET AND BOUNDARY CHECK
                double R7 = ((double)rand()/RAND_MAX) * 1;
                double R8 = ((double)rand()/RAND_MAX) * 1;
                N.A_ICs.at(2)= pP->start_x_dot + R7 - R8;
                if (N.A_ICs.at(2)>pP->start_x_dot_upper_bound){
                    N.A_ICs.at(2) =pP->start_x_dot_upper_bound;
                }
                if (N.A_ICs.at(2)<pP->start_x_dot_lower_bound){
                    N.A_ICs.at(2) =pP->start_x_dot_lower_bound;
                }
            }
            else if (v==3 && random2 <= pP->mutation_rate){
                double R9 = ((double)rand()/RAND_MAX) * 1;
                double R0 = ((double)rand()/RAND_MAX) * 1;
                N.A_ICs.at(3)= pP->displace + R9 - R0;
                if (N.A_ICs.at(3)>(pP->displace_lower_bound + pP->displace_upper_bound)){
                    N.A_ICs.at(3) =pP->displace_lower_bound + pP->displace_upper_bound;
                }
                if (N.A_ICs.at(3)<pP->displace_lower_bound){
                    N.A_ICs.at(3) =pP->displace_lower_bound;
                }
                assert(N.A_ICs.at(3)<=2);
            }
            else{
                assert(random2 > pP->mutation_rate);
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
    for (int j=1; j<pP->num_pol-1;j++){
        assert(test_pro_pol.at(0).P_fitness < test_pro_pol.at(j).P_fitness);
    }
    
}
//-------------------------------------------------------------------------
void EA::update_best_fit(){
    best_P_fitness.push_back(pro_pol.at(0).P_fitness);      // best fitness per generation
    best_A_fitness.push_back(ant_pol.at(0).A_fitness);
    
    if (pP->rand_antagonist==true){
        ofstream AIC_goal,AIC_startx,AIC_startxdot,AIC_displace;
        AIC_goal.open("ANT_goal_history_bestpergen.txt", fstream::app);
        AIC_startx.open("ANT_startx_history_bestpergen.txt", fstream::app);
        AIC_startxdot.open("ANT_startxdot_history_bestpergen.txt", fstream::app);
        AIC_displace.open("ANT_displace_history_bestpergen.txt", fstream::app);
        
        AIC_goal << ant_pol.at(0).A_ICs.at(0) << "\t";  //goalx
        AIC_startx << ant_pol.at(0).A_ICs.at(1) << "\t";  //startx
        AIC_startxdot << ant_pol.at(0).A_ICs.at(2) << "\t";  //startxdot
        AIC_displace << ant_pol.at(0).A_ICs.at(3) << "\t";  //displace
    }
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
    
    if (pP->best_v_median==true){
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
    
    //ANT ICS
    ofstream AIC_goal,AIC_startx,AIC_startxdot;
    AIC_goal.open("ANT_goal_history_bestpergen.txt", fstream::app);
    AIC_startx.open("ANT_startx_history_bestpergen.txt", fstream::app);
    AIC_startxdot.open("ANT_startxdot_history_bestpergen.txt", fstream::app);
    
    AIC_goal << endl;
    AIC_startx << endl;
    AIC_startxdot << endl;
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
    ofstream best_tx,best_txdot,best_txdd, best_Pt_force,best_At_force,fifty_var_test_x;
    
    best_tx.open("test_x_history.txt", fstream::app);
    best_txdot.open("test_x_dot_history.txt",fstream::app);
    best_txdd.open("test_x_dd_history.txt",fstream::app);
    
    best_Pt_force.open("test_P_force_history.txt",fstream::app);
    best_At_force.open("test_A_force_history.txt",fstream::app);
    
    fifty_var_test_x.open("best_50_var_x_history.txt",fstream::app);
    
    ofstream SR_test_best, test_P_fit_hist;
    test_P_fit_hist.open("test_P_best_fitness_history.txt",fstream::app);       //fitness per policy
    SR_test_best.open("test_P_best_fitpergen_SR_history.txt", fstream::app);         //best fitness out of all policies
    
    for (int f =0; f < pP->total_time; f++){
        best_tx << test_pro_pol.at(0).x_history.at(f) << "\t";
        best_txdot << test_pro_pol.at(0).x_dot_history.at(f) << "\t";
        best_txdd << test_pro_pol.at(0).x_dd_history.at(f) << "\t";
        
        best_Pt_force << test_pro_pol.at(0).P_force_history.at(f) << "\t";
        best_At_force << test_ant_pol.at(0).A_force_history.at(f) << "\t";
        
    }
    
    if(pP->multi_var==true){
        for (int z=0;z<pP->num_loops;z++){
            for(int y=0; y<pP->total_time;y++){
                fifty_var_test_x << test_pro_pol.at(0).loop_x_history.at(z).at(y) << "\t";
            }
            fifty_var_test_x << endl;
            
        }
        
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
        test_P_fit_hist << test_pro_pol.at(h).P_fitness << "\t";
        
    }
    test_P_fit_hist << endl;
    SR_test_best << test_pro_pol.at(0).P_fitness << endl;
    
    ofstream med_tx,med_txdot,med_txdd, med_Pt_force,med_At_force, SR_test_med;
    
    med_tx.open("med_test_x_history.txt", fstream::app);
    med_txdot.open("med_test_x_dot_history.txt",fstream::app);
    med_txdd.open("med_test_x_dd_history.txt",fstream::app);
    med_Pt_force.open("med_test_P_force_history.txt",fstream::app);
    med_At_force.open("med_test_A_force_history.txt",fstream::app);
    SR_test_med.open("test_P_med_fitpergen_SR_history.txt", fstream::app);
    
    for (int f =0; f < pP->total_time; f++){
        med_tx << test_pro_pol.at(pro_pol.size()/2).x_history.at(f) << "\t";
        med_txdot << test_pro_pol.at(pro_pol.size()/2).x_dot_history.at(f) << "\t";
        med_txdd << test_pro_pol.at(pro_pol.size()/2).x_dd_history.at(f) << "\t";
        
        med_Pt_force << test_pro_pol.at(pro_pol.size()/2).P_force_history.at(f) << "\t";
        med_At_force << test_ant_pol.at(pro_pol.size()/2).A_force_history.at(f) << "\t";
        
    }
    
    SR_test_med << test_pro_pol.at(pro_pol.size()/2).P_fitness << "\t";
    
    
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
            tstep_position << test_pro_pol.at(h).position_noise_tstep_history.at(j) << "\t";
            //cout << pro_pol.at(h).position_noise_tstep_history.at(j) << "\t";
            tstep_velocity << test_pro_pol.at(h).velocity_noise_tstep_history.at(j) << "\t";
            tstep_sensor << test_pro_pol.at(h).sensor_noise_tstep_history.at(j) << "\t";
            tstep_actuator << test_pro_pol.at(h).actuator_noise_tstep_history.at(j) << "\t";
        }
        tstep_position << endl;
        tstep_velocity << endl;
        tstep_sensor << endl;
        tstep_actuator << endl;
        assert(test_pro_pol.at(h).ave_sensor_noise_history.size() == 1);
        nsensor << test_pro_pol.at(h).ave_sensor_noise_history.at(0) << "\t";
        nactuator << test_pro_pol.at(h).ave_actuator_noise_history.at(0) << "\t";
        nposition << test_pro_pol.at(h).ave_position_noise_history.at(0) << "\t";
        nvelocity << test_pro_pol.at(h).ave_velocity_noise_history.at(0) << "\t";
    }
    nsensor << endl;
    nactuator << endl;
    nposition << endl;
    nvelocity << endl;
    
    
    
}

//-------------------------------------------------------------------------
void EA::Run_Test_Program(){
    test_pro_pol.clear();
    test_ant_pol.clear();
    test_pro_pol = pro_pol;
    test_ant_pol = ant_pol;
    Run_Test_Simulation();
    Evaluate();
    Sort_Test_Policies_By_Fitness();
    
    Graph_test();
}


//-------------------------------------------------------------------------
//Runs the entire program
void EA::Run_Program() {
    ofstream nsensor;
    nsensor.open("ave_sensor_noise.txt", ofstream::out | ofstream::trunc);
    ofstream nactuator;
    nactuator.open("ave_actuator_noise.txt", ofstream::out | ofstream::trunc);
    
    ofstream rand_start,A_fit,P_fit;
    rand_start.open("random_starting_variables.txt", ofstream::out | ofstream::trunc);
    P_fit.open("stat_ave_best_P_fitness.txt", fstream::app);
    A_fit.open("stat_ave_best_A_fitness.txt", fstream::app);
    
    Build_Population();
    //assert(pP->start_x==15);
    best_P_fitness.clear();
    best_A_fitness.clear();
    assert(best_P_fitness.size()==0);
    
    
    for (int gen=0; gen<pP->gen_max; gen++) {
        if (pP->tr_2==true){
            //assert(pP->start_x==15 && pP->start_x_dot==0 && pP->start_x_dd==0);
        }
        if (gen %5 ==0){
            if (pP->rand_start_5gen==true){
                pP->random_start_end_variables();
            }
        }
        if (gen %10 ==0) {
            //cout << "GENERATION \t" << gen << endl;
        }
        if (gen < pP->gen_max-1) {
            
            EA_Process();
            Graph_med();
            
        }
        else {
            
            Run_Simulation();
            Evaluate();
            Sort_Policies_By_Fitness();
            cout << "BEST POLICY PRO-FITNESS" << "\t" << pro_pol.at(0).P_fitness << endl;
            //P_fit << pro_pol.at(0).P_fitness << endl;
            Graph_med();
            
            
        }
        
        

    }
    ave_fit();
    Graph();
    P_fit.close();
    A_fit.close();
    
    nsensor.close();
    nactuator.close();
    
    rand_start.close();
}


#endif /* EA_hpp */
