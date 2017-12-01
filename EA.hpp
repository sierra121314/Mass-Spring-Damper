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
    void Graph();
    void Graph_test();
    double sum;
    double ave;
    int place;
    int num_loops;
    
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
            //double ph = dou);
            //cout << ph << endl;
            double P_r = -1 + (2)*((double)rand()/RAND_MAX);
            double A_r = -1 + (2)*((double)rand()/RAND_MAX);
            //double  r  = -1 + 2*ph;
            pro_pol.at(i).P_weights.push_back(P_r);
            ant_pol.at(i).A_weights.push_back(A_r);
            //cout << r << "\t";
            assert(-1<=P_r && 1>=P_r);
            assert(-1<=A_r && 1>=A_r);
            
        }
        //cout << endl;
        
    }
    assert(pro_pol.size() == pP->num_pol); // check to make sure that the policy sizes are the same
    assert(ant_pol.size() == pP->num_pol);
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
    if (pP->multi_var==true){
        num_loops=50;
        pP->fifty_var();    //initialize 50x3 variables
    }
    else {
        num_loops = 1;
    }
    //LOGGING START POSITIONS
    rand_start << pP->m << "\t" << pP->b << "\t" << pP->k << "\t" << pP->mu << "\t" << pP->start_x << "\t" << pP->goal_x << "\t" << pP->start_x_dot << endl;
    
    for (int i=0; i<pP->num_pol; i++) {
        pro_pol.at(i).P_fitness = 100000000000; //changed from -1 11/15
        ant_pol.at(i).A_fitness = 10000000;
        pro_pol.at(i).P_fit_swap = 0;
        ant_pol.at(i).A_fit_swap = 0;
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
            aPo = & ant_pol.at(i);
            S.Simulate(pPo, aPo);
            
            if (pro_pol.at(i).P_fitness > pro_pol.at(i).P_fit_swap) { //swapped direction 11/15 at 9:50
                pro_pol.at(i).P_fitness = pro_pol.at(i).P_fit_swap;
            }
            if (ant_pol.at(i).A_fitness > ant_pol.at(i).A_fit_swap) {
                ant_pol.at(i).A_fitness = ant_pol.at(i).A_fit_swap;
            }
            assert(pro_pol.at(i).P_fitness>=0);
            assert(ant_pol.at(i).A_fitness>=0 && ant_pol.at(i).A_fitness<10000000);
            /*
            if (pP->multi_var==true) {
                test_fit << pro_pol.at(i).P_fit_swap << "\t";
            }*/
        }
        if (pP->multi_var==true) {
            //test_fit << pro_pol.at(i).P_fit_swap << endl;
            pro_pol.at(i).P_fitness = pro_pol.at(i).P_fit_swap;
        }
    }
    
    if (pP->three_for_three == true) {
        //Uses the same order of Pro policies as before
        //Copy antagonist policies into seperate vector in order to choose best performance
        for (int j=0; j<2; j++) {
            random_shuffle ( ant_pol.begin(), ant_pol.end() );
            
            for (int i=0; i<pP->num_pol; i++) {
                pro_pol.at(i).P_fit_swap = 0;
                ant_pol.at(i).A_fit_swap = 0;
                
                Simulator S;
                S.pP = this->pP;
                Policy* pPo;
                Policy* aPo;
                pPo = & pro_pol.at(i);
                aPo = & ant_pol.at(i);
                S.Simulate(pPo, aPo);
                
                if (pro_pol.at(i).P_fitness > pro_pol.at(i).P_fit_swap) { //swapped direction 11/15 at 9:50
                    pro_pol.at(i).P_fitness = pro_pol.at(i).P_fit_swap;
                }
                if (ant_pol.at(i).A_fitness > ant_pol.at(i).A_fit_swap) {
                    ant_pol.at(i).A_fitness = ant_pol.at(i).A_fit_swap;
                }
                assert(pro_pol.at(i).P_fitness>=0);
                assert(ant_pol.at(i).A_fitness>=0 && ant_pol.at(i).A_fitness<10000000);
            }
            
            
        }
        //find best antagonist set out of 3
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
        if (random2 <= pP->mutation_rate) {
            double R3 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            double R4 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            if (pP->rand_antagonist==false){
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
        for (int y =0; y < 4; y++){
            double random2 = ((double)rand()/RAND_MAX);
            if (random2 <= pP->mutation_rate) {
                double R3 = rand() % 1;
                double R4 = rand() % 1;
                pP->start_x = pP->start_x + R3 - R4;
                pP->start_x_dot = pP->start_x_dot + R3 - R4;
                pP->goal_x = pP->goal_x + R3 - R4;
            
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
    best_P_fitness.push_back(pro_pol.at(0).P_fitness);
    best_A_fitness.push_back(ant_pol.at(0).A_fitness);
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
}
//-------------------------------------------------------------------------
void EA::Graph(){
    ofstream fout;
    //NOTE
    //pro_pol.at(0) is best
    //pro_pol.size()/2 is median
    if (pP->best_v_median==true){
        place = 0; //graph the best
    }
    else {
        place = pro_pol.size()/2; //graph the median
    }
    
    fout.open("x_history.txt", std::ofstream::out | ofstream::trunc);
    //fout << "vector spot" << "\t" << "x" << "\t" << "x_dot" << "\t" << "x_dd" << endl;
    for (int f =0; f < pP->total_time; f++){
        fout << pro_pol.at(place).x_history.at(f) << "\t";
    }
    fout.close();
    
    fout.open("x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    for (int g =0; g < pP->total_time; g++){
        fout << pro_pol.at(place).x_dot_history.at(g) << "\t";
    }
    fout.close();
    
    fout.open("x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << pro_pol.at(place).x_dd_history.at(h) << "\t";
    }
    fout.close();
    fout.open("P_force_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << pro_pol.at(place).P_force_history.at(h) << "\t";
    }
    fout.close();
    fout.open("A_force_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << ant_pol.at(place).A_force_history.at(h) << "\t";
    }
    fout.close();
    fout.open("P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->gen_max; h++){
        fout << best_P_fitness.at(h) << "\t";
    }
    fout.close();
    fout.open("A_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->gen_max; h++){
        fout << best_A_fitness.at(h) << "\t";
    }
    fout.close();
    ofstream SR;
    SR.open("P_best_fitpergen_SR_history.txt", fstream::app);
    for (int h =0; h < pP->gen_max; h++){
        SR << best_P_fitness.at(h) << "\t";
    }
    SR << endl;
    SR.close();
    
}



void EA::Graph_test(){
    ofstream fout;
    
    fout.open("test_x_history.txt", std::ofstream::out | ofstream::trunc);
    //fout << "vector spot" << "\t" << "x" << "\t" << "x_dot" << "\t" << "x_dd" << endl;
    for (int f =0; f < pP->total_time; f++){
        fout << pro_pol.at(0).x_history.at(f) << "\t";
    }
    fout.close();
    
    fout.open("test_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    for (int g =0; g < pP->total_time; g++){
        fout << pro_pol.at(0).x_dot_history.at(g) << "\t";
    }
    fout.close();
    
    fout.open("test_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << pro_pol.at(0).x_dd_history.at(h) << "\t";
    }
    fout.close();
    fout.open("test_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << pro_pol.at(0).P_force_history.at(h) << "\t";
    }
    fout.close();
    
    fout.open("test_P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->num_pol; h++){
        fout << pro_pol.at(h).P_fitness << "\t";

    }
    ofstream SR_test;
    SR_test.open("Ptest_best_fitpergen_SR_history.txt", fstream::app);
    SR_test << best_P_fitness.at(0) << endl;
    SR_test.close();
    
    fout.close();
    
    
}



//-------------------------------------------------------------------------
//Runs the entire program
void EA::Run_Program() {
    ofstream nsensor;
    nsensor.open("ave_sensor_noise.txt", ofstream::out | ofstream::trunc);
    ofstream nactuator;
    nactuator.open("ave_actuator_noise.txt", ofstream::out | ofstream::trunc);
    
    ofstream rand_start;
    rand_start.open("random_starting_variables.txt", ofstream::out | ofstream::trunc);
    
    ofstream P_fit;
    P_fit.open("stat_P_fitness.txt", fstream::app);
    sum=0;
    Build_Population();
    best_P_fitness.clear();
    best_A_fitness.clear();
    for (int gen=0; gen<pP->gen_max; gen++) {
        if (gen %5 ==0){
            if (pP->rand_start_5gen==true){
                pP->random_variables();
            }
        }
        if (gen %10 ==0) {
            //cout << "GENERATION \t" << gen << endl;
        }
        if (gen < pP->gen_max-1) {
            EA_Process();
        }
        else {
            
            Run_Simulation();
            Evaluate();
            Sort_Policies_By_Fitness();
            cout << "BEST POLICY PRO-FITNESS" << "\t" << pro_pol.at(0).P_fitness << endl;
            //P_fit << pro_pol.at(0).P_fitness << endl;
            
            best_P_fitness.push_back(pro_pol.at(0).P_fitness);      // best fitness per generation
            best_A_fitness.push_back(ant_pol.at(0).A_fitness);
            for (int f = 0; f < best_P_fitness.size(); f++) { //help 11/20
                //sum += pro_pol.at(f).P_fitness;
                sum += best_P_fitness.at(f);
            }
            ave = sum/best_P_fitness.size();
            P_fit << ave << endl;
        }
        

    }
    
    Graph();
    nsensor.close();
    nactuator.close();
    
    rand_start.close();
}


#endif /* EA_hpp */
