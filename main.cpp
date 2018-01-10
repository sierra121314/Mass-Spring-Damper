;//
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





int main() {
    srand(time(NULL));
    Parameters P;
    EA E;
    clock_t t1, t2;
    t1 = clock();
    //TRAINING MODES
    P.train_and_test = true; //Determines whether to run a combo of train and test
    
    
    //have an input where you tell it the combo you want
    //so we can put the below stuff into Parameters
    P.five_B = false;
    P.four_B = false;
    P.two_B = false;
    P.three_B = false;
    P.three_A = false;   // FOR research paper keep this TRUE
    P.two_A = true;     // OR keep this TRUE
    P.one = false;
    ofstream test_fit, P_fit, A_fit, SR, SR_test, SR_A, P_testperfive_fit, test_para, P_fit_hist, A_fit_hist, test_P_fit_hist;
    test_fit.open("stat_Ptest_fitness.txt", ofstream::out | ofstream::trunc);
    P_fit.open("stat_ave_best_P_fitness.txt", ofstream::out | ofstream::trunc);
    A_fit.open("stat_ave_best_A_fitness.txt", ofstream::out | ofstream::trunc);
    SR.open("P_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);                    //best fitness per generation for all SR
    SR_A.open("A_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);
    SR_test.open("test_P_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);          //best fitness out of all policies for all SR
    P_fit_hist.open("P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);             //overall best fitness per stat run
    test_P_fit_hist.open("test_P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);   //best fitness per policy of single gen
    A_fit_hist.open("A_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);
    P_testperfive_fit.open("stat_P_testperfive_fit.txt", ofstream::out | ofstream::trunc);
    
    
    
    for (int s=0; s < P.stat_runs; s++){
        if (P.train_and_test == true){
            P.test_train_set();
            P.three_for_three = false; //change this one
            P.train();
            E.pP = &P;
            E.Run_Program();
            if(P.te_1==true || P.te_A==true || P.te_B==true){
                ofstream nsensor, nactuator, nposition, nvelocity, tstep_sensor, tstep_actuator,tstep_position, tstep_velocity;
                nsensor.open("ave_sensor_noise.txt", ofstream::out | ofstream::trunc);
                nactuator.open("ave_actuator_noise.txt", ofstream::out | ofstream::trunc);
                nposition.open("ave_position_noise.txt", ofstream::out | ofstream::trunc);
                nvelocity.open("ave_velocity_noise.txt", ofstream::out | ofstream::trunc);
                tstep_sensor.open("tstep_sensor.txt", ofstream::out | ofstream::trunc);
                tstep_actuator.open("tstep_actuator.txt", ofstream::out | ofstream::trunc);
                tstep_position.open("tstep_position.txt", ofstream::out | ofstream::trunc);
                tstep_velocity.open("tstep_velocity.txt", ofstream::out | ofstream::trunc);
                
                P.test();
                //E.pP = &P;
                E.Run_Simulation(); //Should this be Run_Test_Simulation();
                E.Evaluate();
                E.Sort_Policies_By_Fitness();//Sort_Test_Policies_By_Fitness()
                cout << "BEST POLICY Test PRO-FITNESS" << "\t" << E.pro_pol.at(0).P_fitness << endl;

                test_fit << E.pro_pol.at(0).P_fitness << endl;
                E.Graph_test();
                nsensor.close();
                nactuator.close();
                tstep_sensor.close();
                tstep_actuator.close();
                tstep_position.close();
                tstep_velocity.close();
                
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
    t2 = clock();
    float diff ((float)t2-(float)t1);
    float seconds = diff / CLOCKS_PER_SEC;
    cout << "run time" << "\t" << seconds << endl;
    
    test_para.open("testing_parameters.txt", fstream::app);
    test_para << "run time" << "\t" << seconds << endl;
    test_fit.close();
    P_fit.close();
    A_fit.close();
    P_fit_hist.close();
    A_fit_hist.close();
    test_P_fit_hist.close();
    SR.close();
    SR_A.close();
    SR_test.close();
    P_testperfive_fit.close();
    test_para.close();
    cout << "END PROGRAM" << endl;
}
