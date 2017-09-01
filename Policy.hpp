//
//  Policy.hpp
//  Pro-Ant_EA_project
//
//  Created by Sierra Gonzales on 8/21/17.
//  Copyright © 2017 Pro-Ant_EA_project. All rights reserved.
//

#ifndef Policy_hpp
#define Policy_hpp

#include <stdio.h>

using namespace std;


class Policy
{
    friend class EA;
    friend class Parameters;
    friend class Simulator;
    
protected:
    
    
public:

    //Include all needed information about any policy here
    vector<double> weights;
    double fitness;
    int age; //how long did it last?
    int x; // position of mass
    int x_dot; //velocity
    int x_dd; //acceleration
    int m = 5; //mass
    int b = 1; //damper
    int k = 1; //spring
    int dt = 0.01; //time step [s]
    
    int mu = 0; //friction
    int P_force = 1; //Protagonist force
    
    int desired_x;
    int desired_x_dot = 0;
    int desired_x_dd = 0;
    
    void Init_P_policy(int num_weights); //initializes one policy
    
private:
};

void Policy::Init_P_policy(int num_weights) {
    for (int p = 0; p < num_weights; p++) {
        //cout << "Order " << p << endl;
        weights.push_back(0);
        
    }
    
    
}

#endif /* Policy_hpp */
