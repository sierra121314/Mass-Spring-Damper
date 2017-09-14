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
    friend class neural_network;
    
protected:
    
    
public:

    //Include all needed information about any policy here
    vector<double> weights;
    vector<double> A_weights;
    vector<double> x_history;
    vector<double> x_dot_history;
    vector<double> x_dd_history;
    vector<double> P_force_history;
    //...
    double fitness;
    int age; //how long did it last?
    
    double x; // position of mass
    double x_dot; //velocity
    double x_dd; //acceleration
    double m = 7; //mass
    double b = -1; //damper
    double k = 1; //spring
    double dt = 0.1; //time step [s]
    
    double mu = 0; //friction
    double P_force; //Protagonist force

    double desired_x = 0;
    double desired_x_dot = 0;
    double desired_x_dd = 0;
    
    void Init_P_policy(int num_weights); //initializes one policy
    void Init_A_policy(int num_A_weights);
    
    void Calc_P_fitness();
    
private:
};

void Policy::Init_P_policy(int num_weights) {
    for (int p = 0; p < num_weights; p++) {
        //cout << "Order " << p << endl;
        weights.push_back(0);
        
    }
    
    
}
/*
void Policy::Init_A_policy(<#int num_A_weights#>) {
    for(int pa = 0; pa < num_A_weights; pa++) {
        //cout << "Order " << pa << endl;
        A_weights.push_back(0);
        
    }
}
*/




#endif /* Policy_hpp */
