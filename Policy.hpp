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
    vector<double> A_weights;
    double fitness;
    int age; //how long did it last?
    
    double x; // position of mass
    double x_dot; //velocity
    double x_dd; //acceleration
    double m = 7; //mass
    double b = 1; //damper
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
/*
void Policy::Calc_P_fitness(){
    int F_dist = (abs(pPo->start_x - x)) * 100; //want closest to 0 displacement
    int F_vel = abs(desired_x_dot - x_dot) * 10; // want smallest velocity
    int F_acc = abs(desired_x_dd - x_dd) * 10; // want smallest acceleration
    //policy.at(i).fitness = F_dist + F_vel + F_acc; //total fitness
    fitness = F_dist + F_vel + F_acc;
    //fitness.push_back(fitness); //vector of fitness?
}
*/
#endif /* Policy_hpp */
