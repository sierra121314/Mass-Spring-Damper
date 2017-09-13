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
    
    vector<Policy> pol;

    void Build_Population();
    void Run_Simulation();
    void Evaluate();
    int Binary_Select();
    void Downselect();
    void Mutation(Policy &M);
    void Repopulate();
    struct Less_Than_Policy_Fitness;
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
    neural_network ANN;
    ANN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs); //3 input, 5 hidden, 1 output
    pP->num_weights = ANN.intended_size;
    //cout << pP->num_weights << endl;
    
    for (int i=0; i<pP->num_pol; i++)
    {
        Policy P;
        pol.push_back(P);
        pol.at(i).age = 0;
        pol.at(i).fitness = 0;
        //cout << "Policy" << "\t" << i << "\t" << "weights" << "\t";
        //for that policy have a vector of weights
        for (int w=0; w < pP->num_weights; w++){
            //pick random # between 0 and 1 and put into that vector
            //double ph = dou);
            //cout << ph << endl;
            double r = -1 + (2)*((double)rand()/RAND_MAX);
            //double  r  = -1 + 2*ph;
            pol.at(i).weights.push_back(r);
            //cout << r << "\t";
            assert(-1<=r && 1>=r);
            
        }
        //cout << endl;
        
    }
    assert(pol.size() == pP->num_pol); // check to make sure that the policy sizes are the same
}


//////////////////////////////////////////////////////////////////////////////
//Puts each policy into the simulation
void EA::Run_Simulation()
{
    for (int i=0; i<pP->num_pol; i++)
    {
        pol.at(i).fitness = 0;
        //First we insert a policy into the simulator then we can take the objective data for that policy and store it in our data architecture
        Simulator S;

        S.pP = this->pP;
        Policy* pPo;
        pPo = & pol.at(i);
        S.Simulate(pPo);
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
int EA::Binary_Select()
{
    int loser;
    int index_1 = rand() % pol.size();
    int index_2 = rand() % pol.size();
    while (index_1 == index_2)
    {
        index_2 = rand() % pol.size();
    }
    
    //winner is one with lower fitness
    //Can be switched for a maximization problem
    if(pol.at(index_1).fitness < pol.at(index_2).fitness)
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


//-------------------------------------------------------------------------
//Policies are compared to determine the optimal policies for a given generation
void EA::Downselect()
{
    int kill;
    for(int k=0; k<pP->to_kill; k++)
    {
        kill = Binary_Select();
        pol.erase(pol.begin() + kill);
    }
    assert(pol.size() == pP->to_kill);
    for(int i=0; i<pP->to_kill; i++)
    {
        pol.at(i).age += 1;
    }
}


//-------------------------------------------------------------------------
//Mutates the copies of the winning individuals
void EA::Mutation(Policy &M)
{
   //This is where the policy is slightly mutated
    
    
    for (int x = 0; x < pP->num_weights; x++)
    {
        double random = ((double)rand()/RAND_MAX);
        //cout << "r" << "\t" << random << endl;
        if (random <= pP->mutation_rate)
        {
            double R1 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            double R2 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            M.weights.at(x) = M.weights.at(x) + (R1-R2);
            if (M.weights.at(x)<-1)
            {
                M.weights.at(x) = -1;
            }
            if (M.weights.at(x)>1)
            {
                M.weights.at(x) = 1;
            }
            //cout << x << "\t";
        }
        //cout << Gen.at(Gen.size()-1).weights.at(x) << endl;
        assert(M.weights.at(x)<=1 && M.weights.at(x)>=-1);
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
        int spot = rand() % pol.size();
        M = pol.at(spot);
        //cout << "cp" << endl;
        Mutation(M);
        pol.push_back(M);
        pol.at(pol.size()-1).age = 0; //how long it has survived
    }
    assert(pol.size() == pP->num_pol);
}


//-------------------------------------------------------------------------
//Runs the entire EA loop process
void EA::EA_Process()
{
    Run_Simulation();
    Evaluate();
    Downselect();
    Repopulate();
}


//-------------------------------------------------------------------------
//Sorts the population based on their fitness from lowest to highest
struct EA::Less_Than_Policy_Fitness
{
    inline bool operator() (const Policy& struct1, const Policy& struct2)
    {
        return (struct1.fitness < struct2.fitness);
    }
};


//-------------------------------------------------------------------------
//Sorts population
void EA::Sort_Policies_By_Fitness()
{
    for (int i=0; i<pP->num_pol; i++)
    {
        sort(pol.begin(), pol.end(), Less_Than_Policy_Fitness());
    }
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
            Sort_Policies_By_Fitness();
            cout << "BEST POLICY FITNESS" << "\t" << pol.at(0).fitness << endl;
        }
        else
        {
            cout << "FINAL GENERATION" << endl;
            Run_Simulation();
            Evaluate();
            Sort_Policies_By_Fitness();
            cout << "BEST POLICY FITNESS" << "\t" << pol.at(0).fitness << endl;
            
        }
    }
    //fout << "vector spot" << "\t" << "x" << "\t" << "x_dot" << "\t" << "x_dd" << endl;
    for (int f =0; f < pP->total_time; f++){
        fout << pol.at(0).x_history.at(f) << "\t";
    }
    fout.close();
    
    fout.open("x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    for (int g =0; g < pP->total_time; g++){
        fout << pol.at(0).x_dot_history.at(g) << "\t";
    }
    fout.close();
    
    fout.open("x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << pol.at(0).x_dd_history.at(h) << "\t";
    }
    fout.close();
    fout.open("P_force_history.txt",std::ofstream::out | ofstream::trunc);
    for (int h =0; h < pP->total_time; h++){
        fout << pol.at(0).P_force_history.at(h) << "\t";
    }
    fout.close();
}


#endif /* EA_hpp */
