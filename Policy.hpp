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
    double x; // position of mass
    double x_dot; //velocity
    double x_dd; //acceleration
    double m = 5; //mass
    double b = 1; //damper
    double k = 1; //spring
    double dt = 0.01; //time step [s]
    
    double mu = .5; //friction
    double P_force; //Protagonist force
    
    double desired_x;
    double desired_x_dot = 0;
    double desired_x_dd = 0;
    
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
