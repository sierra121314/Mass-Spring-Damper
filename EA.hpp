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

using namespace std;


class EA
{
    friend class Parameters;
    friend class Simulator;
    friend class Policy;
    
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
    
private:
};


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------
//Builds population of policies
void EA::Build_Population()
{
    for (int i=0; i<pP->num_pol; i++)
    {
        Policy P;
        pol.push_back(P);
        //for that policy have a vector of weights
        //pick random # between 0 and 1 and put into that vector
        //r = random #
        //for loop for how many weights in that vector
        //pol.at(i).weights.push_back(1);
    }
    assert(pol.size() == pP->num_pol);
}


//-------------------------------------------------------------------------
//Puts each policy into the simulation
void EA::Run_Simulation()
{
    for (int i=0; i<pP->num_pol; i++)
    {
        //First we insert a policy into the simulator then we can take the objective data for that policy and store it in our data architecture
        Simulator S;
        Parameters P;
        S.pP = &P;
        Policy* pPo;
        pPo = & pol.at(i);
        S.Simulate(pPo);
    }
}


//-------------------------------------------------------------------------
//Evaluates each policies fitness scores for each objective
void EA::Evaluate()
{
    //might not need
    for (int i=0; i< pP->num_pol; i++)
    {
        
    }
}


//-------------------------------------------------------------------------
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
    
    /*
    for (int j=0; j<pP->num_x_val; j++)
    {
        double r = ((double)rand()/RAND_MAX);
        if (r <= pP->mutation_rate)
        {
            double R1 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            double R2 = ((double)rand()/RAND_MAX) * pP->mutate_range;
            //M.x_val.at(j) = M.x_val.at(j) + (R1-R2);
            
        }
        //assert(M.x_val.at(j) >= pP->x_val_min && M.x_val.at(j) <= pP->x_val_max);
    }
     */
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
    Build_Population();
    for (int gen=0; gen<pP->gen_max; gen++)
    {
        if (gen < pP->gen_max-1)
        {
            EA_Process();
            Sort_Policies_By_Fitness();
        }
        else
        {
            Run_Simulation();
            Evaluate();
            Sort_Policies_By_Fitness();
        }
    }
}


#endif /* EA_hpp */