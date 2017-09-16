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
    
    
    //int num_weights = 5;
    
private:
};


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//////////////////////////////////////////////////////////////////////////////
//Builds population of policies
void EA::Build_Population()
{
    neural_network PNN;
    PNN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs);
    pP->num_weights = PNN.intended_size;
    //cout << pP->num_weights << endl;
    
    for (int i=0; i<pP->num_pol; i++)
    {
        Policy proP;
        Policy antP;
        pro_pol.push_back(proP);
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
void EA::Run_Simulation()
{
    random_shuffle ( pro_pol.begin(), pro_pol.end() );
    random_shuffle ( ant_pol.begin(), ant_pol.end() );
    for (int i=0; i<pP->num_pol; i++)
    {
        pro_pol.at(i).P_fitness = 0;
        ant_pol.at(i).A_fitness = 0;
        //First we insert a policy into the simulator then we can take the objective data for that policy and store it in our data architecture
        Simulator S;

        S.pP = this->pP;
        Policy* pPo;
        Policy* aPo;
        pPo = & pro_pol.at(i);
        aPo = & ant_pol.at(i);
        S.Simulate(pPo, aPo);
    }
}


//////////////////////////////////////////////////////////////////////////////
//Evaluates each policies fitness scores for each objective
void EA::Evaluate()
{
    //might not need
    for (int i=0; i< pP->num_pol; i++)
    {
      
        
    }
}


//////////////////////////////////////////////////////////////////////////////
//Randomly selects two individuals and decides which one will die based on their fitness
int EA::P_Binary_Select()
{
    int loser;
    int index_1 = rand() % pro_pol.size();
    int index_2 = rand() % pro_pol.size();
    while (index_1 == index_2)
    {
        index_2 = rand() % pro_pol.size();
    }
    
    //winner is one with lower fitness
    if(pro_pol.at(index_1).P_fitness < pro_pol.at(index_2).P_fitness)
    {
        loser = index_2;
        //cout << "loser" << "\t" <<  "agent" << "\t" << index_2 << endl;
    }
    else
    {
        loser = index_1;
        //cout << "loser" << "\t" <<  "agent" << "\t" << index_1 << endl;
    }
    return loser;
}
int EA::A_Binary_Select()
{
    int a_loser;
    int index_3 = rand() % ant_pol.size();
    int index_4 = rand() % ant_pol.size();
    while (index_3 == index_4)
    {
        index_4 = rand() % ant_pol.size();
    }
    
    //winner is one with higher fitness
    if(ant_pol.at(index_3).A_fitness > ant_pol.at(index_4).A_fitness)
    {
        a_loser = index_4;
        //cout << "loser" << "\t" <<  "agent" << "\t" << index_2 << endl;
    }
    else
    {
        a_loser = index_3;
        //cout << "loser" << "\t" <<  "agent" << "\t" << index_1 << endl;
    }
    return a_loser;
}



//-------------------------------------------------------------------------
//Policies are compared to determine the optimal policies for a given generation
void EA::Downselect()
{
    int pro_kill;
    int ant_kill;
    for(int k=0; k<pP->to_kill; k++)
    {
        pro_kill = P_Binary_Select();               //Protagonist
        pro_pol.erase(pro_pol.begin() + pro_kill);
        ant_kill = A_Binary_Select();               //Antagonist
        ant_pol.erase(ant_pol.begin() + ant_kill);
    }
    assert(pro_pol.size() == pP->to_kill);
    assert(ant_pol.size() == pP->to_kill);
    for(int i=0; i<pP->to_kill; i++)
    {
        pro_pol.at(i).age += 1;
        ant_pol.at(i).age += 1;
    }
}


//-------------------------------------------------------------------------
//Mutates the copies of the winning individuals
void EA::Mutation(Policy &M, Policy &N)
{
   //This is where the policy is slightly mutated
    
    
    for (int x = 0; x < pP->num_weights; x++)
    {
        double random = ((double)rand()/RAND_MAX);
        double random2 = ((double)rand()/RAND_MAX);
        //cout << "r" << "\t" << random << endl;
        if (random <= pP->mutation_rate) {                          // Protagonist
            double R1 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            double R2 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            M.P_weights.at(x) = M.P_weights.at(x) + (R1-R2);
            if (M.P_weights.at(x)<-1) {
                M.P_weights.at(x) = -1;
            }
            if (M.P_weights.at(x)>1) {
                M.P_weights.at(x) = 1;
            }
            //cout << x << "\t";
        }
        if (random2 <= pP->mutation_rate) {                         // Antagonist
            double R3 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            double R4 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            N.A_weights.at(x) = N.A_weights.at(x) + (R3-R4);
            if (N.A_weights.at(x)<-1) {
                N.A_weights.at(x) = -1;
            }
            if (N.A_weights.at(x)>1) {
                N.A_weights.at(x) = 1;
            }
            //cout << x << "\t";
        }
        //cout << Gen.at(Gen.size()-1).weights.at(x) << endl;
        assert(M.P_weights.at(x)<=1 && M.P_weights.at(x)>=-1);
        assert(N.A_weights.at(x)<=1 && N.A_weights.at(x)>=-1);
    }
    
}


//-------------------------------------------------------------------------
//The population is repopulated based on small mutations of the remaining policies
void EA::Repopulate()
{
    int to_replicate = pP->to_kill;
    for (int rep=0; rep<to_replicate; rep++)
    {
        Policy M;
        Policy N;
        int spot = rand() % pro_pol.size();
        int spot2 = rand() % ant_pol.size();
        M = pro_pol.at(spot);
        N = ant_pol.at(spot2);
        //cout << "cp" << endl;
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
void EA::EA_Process()
{
    Run_Simulation();
    Evaluate();
    Sort_Policies_By_Fitness();
    //cout << "BEST POLICY PRO-FITNESS" << "\t" << pro_pol.at(0).P_fitness << endl;
    //cout << "BEST POLICY ANT-FITNESS" << "\t" << ant_pol.at(0).A_fitness << endl;
    best_P_fitness.push_back(pro_pol.at(0).P_fitness);
    best_A_fitness.push_back(ant_pol.at(0).A_fitness);
    Downselect();
    Repopulate();
}


//-------------------------------------------------------------------------
//Sorts the population based on their fitness from lowest to highest
struct EA::Less_Than_Policy_Fitness
{
    inline bool operator() (const Policy& struct1, const Policy& struct2)
    {
        return (struct1.P_fitness < struct2.P_fitness);
    }
};

struct EA::Greater_Than_Policy_Fitness
{
    inline bool operator() (const Policy& struct3, const Policy& struct4)
    {
        return (struct3.A_fitness > struct4.A_fitness);
    }
};

//-------------------------------------------------------------------------
//Sorts population
void EA::Sort_Policies_By_Fitness()
{
    for (int i=0; i<pP->num_pol; i++)
    {
        sort(pro_pol.begin(), pro_pol.end(), Less_Than_Policy_Fitness());
        sort(ant_pol.begin(), ant_pol.end(), Greater_Than_Policy_Fitness());
    }
    /*
     for (int i=0; i<pP->num_pol; i++)
     {
     sort(ant_pol.begin(), ant_pol.end(), Less_Than_Policy_Fitness());
     }
     
     */
}


//-------------------------------------------------------------------------
//Runs the entire program
void EA::Run_Program()
{
    ofstream fout;
    fout.open("x_history.txt", std::ofstream::out | ofstream::trunc);
    Build_Population();
    for (int gen=0; gen<pP->gen_max; gen++)
    {
        cout << "GENERATION \t" << gen << endl;
        if (gen < pP->gen_max-1)
        {
            EA_Process();
            
        }
        else
        {
            cout << "FINAL GENERATION" << endl;
            Run_Simulation();
            Evaluate();
            Sort_Policies_By_Fitness();
            cout << "BEST POLICY PRO-FITNESS" << "\t" << pro_pol.at(0).P_fitness << endl;
            best_P_fitness.push_back(pro_pol.at(0).P_fitness);      // best fitness per generation
            best_A_fitness.push_back(ant_pol.at(0).A_fitness);
            
        }
    }
    //fout << "vector spot" << "\t" << "x" << "\t" << "x_dot" << "\t" << "x_dd" << endl;
    for (int f =0; f < pP->total_time; f++){
        fout << pro_pol.at(0).x_history.at(f) << "\t";
    }
    fout.close();
    
    fout.open("x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    for (int g =0; g < pP->total_time; g++){
        fout << pro_pol.at(0).x_dot_history.at(g) << "\t";
    }
    fout.close();
    
    fout.open("x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << pro_pol.at(0).x_dd_history.at(h) << "\t";
    }
    fout.close();
    fout.open("P_force_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << pro_pol.at(0).P_force_history.at(h) << "\t";
    }
    fout.close();
    fout.open("A_force_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << ant_pol.at(0).A_force_history.at(h) << "\t";
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
}


#endif /* EA_hpp */
