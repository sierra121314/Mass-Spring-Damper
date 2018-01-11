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
    ofstream test_fit, P_fit, A_fit, SR, SR_test, SR_A, P_testperfive_fit, test_para, P_fit_hist, A_fit_hist;
    test_fit.open("stat_Ptest_fitness.txt", ofstream::out | ofstream::trunc);
    P_fit.open("stat_ave_best_P_fitness.txt", ofstream::out | ofstream::trunc);
    A_fit.open("stat_ave_best_A_fitness.txt", ofstream::out | ofstream::trunc);
    SR.open("P_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);                    //best fitness per generation for all SR
    SR_A.open("A_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);
    SR_test.open("test_P_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);          //best fitness out of all policies for all SR
    P_fit_hist.open("P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);             //overall best fitness per stat run
    A_fit_hist.open("A_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);
    P_testperfive_fit.open("stat_P_testperfive_fit.txt", ofstream::out | ofstream::trunc);
    
    
    // TEST //
    ofstream fout,best_x,best_xdot,best_xdd, best_P_force,best_A_force,SR_test_best, test_P_fit_hist;
    best_x.open("test_x_history.txt", std::ofstream::out | ofstream::trunc);
    best_xdot.open("test_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    best_xdd.open("test_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    best_P_force.open("test_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    best_A_force.open("test_A_force_history.txt",std::ofstream::out | ofstream::trunc);
    test_P_fit_hist.open("test_P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);               //best fitness per policy of single gen
    SR_test_best.open("test_P_best_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);           //best fitness out of all policies
    
    ofstream med_x,med_xdot,med_xdd, med_P_force,med_A_force, SR_test_med;
    med_x.open("med_test_x_history.txt", std::ofstream::out | ofstream::trunc);
    med_xdot.open("med_test_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    med_xdd.open("med_test_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    med_P_force.open("med_test_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    med_A_force.open("med_test_A_force_history.txt",std::ofstream::out | ofstream::trunc);
    SR_test_med.open("test_P_med_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);
    
    ofstream nsensor, nactuator, nposition, nvelocity, tstep_sensor, tstep_actuator,tstep_position, tstep_velocity;
    nsensor.open("ave_sensor_noise.txt", ofstream::out | ofstream::trunc);
    nactuator.open("ave_actuator_noise.txt", ofstream::out | ofstream::trunc);
    nposition.open("ave_position_noise.txt", ofstream::out | ofstream::trunc);
    nvelocity.open("ave_velocity_noise.txt", ofstream::out | ofstream::trunc);
    tstep_sensor.open("tstep_sensor.txt", ofstream::out | ofstream::trunc);
    tstep_actuator.open("tstep_actuator.txt", ofstream::out | ofstream::trunc);
    tstep_position.open("tstep_position.txt", ofstream::out | ofstream::trunc);
    tstep_velocity.open("tstep_velocity.txt", ofstream::out | ofstream::trunc);
    
    for (int s=0; s < P.stat_runs; s++){
            EA E;
        if (P.train_and_test == true){
            P.test_train_set();
            P.three_for_three = false; //change this one
            P.train();
            E.pP = &P;
            E.Run_Program();
            if(P.te_1==true || P.te_A==true || P.te_B==true){
                
                
                P.test();
                //E.pP = &P;
                E.Run_Simulation(); //Should this be Run_Test_Simulation();
                E.Evaluate();
                E.Sort_Policies_By_Fitness();//Sort_Test_Policies_By_Fitness()
                cout << "BEST POLICY Test PRO-FITNESS" << "\t" << E.pro_pol.at(0).P_fitness << endl;

                test_fit << E.pro_pol.at(0).P_fitness << endl;
                E.Graph_test();

                
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
    
    nsensor.close();
    nactuator.close();
    tstep_sensor.close();
    tstep_actuator.close();
    tstep_position.close();
    tstep_velocity.close();
    cout << "END PROGRAM" << endl;
}
