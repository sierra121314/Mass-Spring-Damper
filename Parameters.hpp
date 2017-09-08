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
    int gen_max = 50;                   //number of generations
    double mutation_rate = 0.5;         //mutation rate
    double mutate_range = 0.05;          //mutation range
    
    
    
    bool only_pro = true;    //Just protagonist?
    
    
    
private:
};

#endif /* Parameters_hpp */
