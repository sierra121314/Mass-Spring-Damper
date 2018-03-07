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


int stat_runs = 5;


int main() {
    srand(time(NULL));
    clock_t t1, t2,t3;
    t1 = clock();
    
    ofstream test_fit, P_fit, A_fit, SR, SR_test,test_para;
    test_fit.open("stat_Ptest_fitness.txt", ofstream::out | ofstream::trunc);
    P_fit.open("stat_P_fitness.txt", ofstream::out | ofstream::trunc);
    A_fit.open("stat_ave_best_A_fitness.txt", ofstream::out | ofstream::trunc);
    
    SR.open("P_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);
    SR_test.open("Ptest_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);
    Parameters P;
    P.trunc_Graphs();
    
    
    for (int s=0; s<stat_runs; s++){
        Parameters P;
        
        //TRAINING MODES
        P.train_and_test = true; //CHANGE
        
        P.five_B = true;
        P.four_B = false;
        P.two_B = false;
        P.three_B = false;
        P.three_A = false;
        P.two_A = false;
        P.one = false;
        if (P.two_B==true){
            P.five_B = false;
            P.four_B = false;
            P.three_B = false;
            P.three_A = false;
            P.two_A = false;
            P.one = false;
        }
        if (P.five_B == true){
            P.four_B = false;
            P.two_B = false;
            P.three_B = false;
            P.three_A = false;
            P.two_A = false;
            P.one = false;
        }
        
        EA E;
        if (P.train_and_test == true){
            P.test_train_set();
            P.three_for_three = false; //change this one
            P.train();
            E.pP = &P;
            E.Run_Program();
            if(P.te_1==true || P.te_A==true || P.te_B==true){
                assert(P.te_1==true || P.te_A==true || P.te_B==true);
                P.test();
                //E.pP = &P;
                assert(P.A_f_max_bound==0);
                
                E.Run_Test_Program();
                
                cout << "BEST POLICY Test PRO-FITNESS" << "\t" << E.test_pro_pol.at(0).P_fitness << endl;
                
                test_fit << E.test_pro_pol.at(0).P_fitness << endl;
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
        t3 = clock();
        float df ((float)t3-(float)t1);
        float sec = df /CLOCKS_PER_SEC;
        cout << "SR time\t" << sec << endl;
    }
    t2 = clock();
    float diff ((float)t2-(float)t1);
    float seconds = diff / CLOCKS_PER_SEC;
    cout << "run time" << "\t" << seconds << endl;
    
    test_para.open("testing_parameters.txt", fstream::app);
    test_para << "run time" << "\t" << seconds << endl;
    
    //P.closeGraphs();
    
    test_fit.close();
    P_fit.close();
    SR.close();
    SR_test.close();
    cout << "END PROGRAM" << endl;
}
