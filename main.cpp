//
//  main.cpp
//  Pro-Ant_EA_project
//
//  Created by Sierra Gonzales on 8/21/17.
//  Copyright © 2017 Pro-Ant_EA_project. All rights reserved.
//

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



#include "Parameters.hpp"
#include "Policy.hpp"
#include "Simulator.hpp"
#include "EA.hpp"

bool three;
bool two;
bool one;
int stat_runs = 30;


int main()
{
    srand(time(NULL));
    Parameters P;
    EA E;
    
    //TRAINING MODES
    P.train_and_test = true; //CHANGE
    
    three = true;
    two = false;
    one = false;
    ofstream test_fit;
    test_fit.open("stat_Ptest_fitness.txt", ofstream::out | ofstream::trunc);
    ofstream P_fit;
    P_fit.open("stat_P_fitness.txt", ofstream::out | ofstream::trunc);

    for (int s=0; s<stat_runs; s++){
        
        if (one ==true){
            P.tr_1 = true;
            P.te_1 = true;
            cout << "test one" <<endl;
        }
        if (two == true){
            P.tr_2 = true;
            P.te_2 = true;
            cout << "test two" <<endl;
        }
        if (three == true){
            P.tr_3 = true;
            P.te_3 = true;
            cout << "test three" <<endl;
        }
        
        
        if (P.train_and_test == true){
            P.train();
            E.pP = &P;
            E.Run_Program();
            if(P.te_1==true || P.te_2==true || P.te_3==true){
                P.test();
                E.Run_Simulation();
                E.Evaluate();
                E.Sort_Policies_By_Fitness();
                cout << "BEST POLICY Test PRO-FITNESS" << "\t" << E.pro_pol.at(0).P_fitness << endl;
                test_fit << E.pro_pol.at(0).P_fitness << endl;
                E.Graph_test();
                //run nn that is trained but don't evolve any further
                //1 simulation of the best
                //1 simulation of medium of population
            }
        }
        
        else {
            P.P_f_min_bound = -5;
            P.P_f_max_bound = 5;
            P.A_f_min_bound = -1;
            P.A_f_max_bound = 1;
            
            E.pP = &P;
            E.Run_Program();
            
        }

    }
    test_fit.close();
    P_fit.close();
    
    cout << "END PROGRAM" << endl;
}
