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

    clock_t t1, t2,t3;
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
    ofstream test_fit, P_fit, A_fit, P_testperfive_fit, test_para;
    test_fit.open("stat_Ptest_fitness.txt", ofstream::out | ofstream::trunc);
    P_fit.open("stat_ave_best_P_fitness.txt", ofstream::out | ofstream::trunc);
    A_fit.open("stat_ave_best_A_fitness.txt", ofstream::out | ofstream::trunc);
    P_testperfive_fit.open("stat_P_testperfive_fit.txt", ofstream::out | ofstream::trunc);
    
    // TRAIN //
    ofstream x_best,xdot_best,xdd_best, best_P_force,best_A_force, P_fit_hist, A_fit_hist, SR, SR_A;
    x_best.open("x_history.txt", std::ofstream::out | ofstream::trunc);
    xdot_best.open("x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    xdd_best.open("x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    best_P_force.open("P_force_history.txt",std::ofstream::out | ofstream::trunc);
    best_A_force.open("A_force_history.txt",std::ofstream::out | ofstream::trunc);
    P_fit_hist.open("P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);             //overall best fitness per stat run
    A_fit_hist.open("A_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);
    SR.open("P_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);                    //best fitness per generation for all SR
    SR_A.open("A_best_fitpergen_SR_history.txt", ofstream::out | ofstream::trunc);
    
    ofstream x_med,xdot_med,xdd_med, med_P_force,med_A_force, SR_med, SR_A_med;
    x_med.open("med_x_history.txt", std::ofstream::out | ofstream::trunc);
    xdot_med.open("med_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    xdd_med.open("med_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    med_P_force.open("med_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    med_A_force.open("med_A_force_history.txt",std::ofstream::out | ofstream::trunc);
    SR_med.open("P_med_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);       //best fitness per generation
    SR_A_med.open("A_med_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);
    
    // TEST //
    ofstream fout,best_tx,best_txdot,best_txdd, best_Pt_force,best_At_force,SR_test_best, test_P_fit_hist;
    best_tx.open("test_x_history.txt", std::ofstream::out | ofstream::trunc);
    best_txdot.open("test_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    best_txdd.open("test_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    best_Pt_force.open("test_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    best_At_force.open("test_A_force_history.txt",std::ofstream::out | ofstream::trunc);
    test_P_fit_hist.open("test_P_best_fitness_history.txt",std::ofstream::out | ofstream::trunc);               //best fitness per policy of single gen
    SR_test_best.open("test_P_best_fitpergen_SR_history.txt", std::ofstream::out | ofstream::trunc);           //best fitness out of all policies
    
    ofstream med_tx,med_txdot,med_txdd, med_Pt_force,med_At_force, SR_test_med;
    med_tx.open("med_test_x_history.txt", std::ofstream::out | ofstream::trunc);
    med_txdot.open("med_test_x_dot_history.txt",std::ofstream::out | ofstream::trunc);
    med_txdd.open("med_test_x_dd_history.txt",std::ofstream::out | ofstream::trunc);
    med_Pt_force.open("med_test_P_force_history.txt",std::ofstream::out | ofstream::trunc);
    med_At_force.open("med_test_A_force_history.txt",std::ofstream::out | ofstream::trunc);
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
                assert(P.A_f_max_bound==0);
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
    
    best_tx.close();
    best_txdot.close();
    best_txdd.close();
    best_Pt_force.close();
    best_At_force.close();
    
    x_med.close();
    xdot_med .close();
    xdd_med .close();
    med_P_force.close();
    med_A_force.close();
    SR_med.close();
    SR_A_med.close();
    
    test_fit.close();
    P_fit.close();
    A_fit.close();
    P_fit_hist.close();
    A_fit_hist.close();
    test_P_fit_hist.close();
    SR.close();
    SR_A.close();
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
