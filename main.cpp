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


int stat_runs = 2;


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
        
        P.five_A = false;
        P.five_B = false;
        P.four_A = false;
        P.four_B = false;
        P.two_B = false;
        P.three_B = false;
        P.three_A = true;
        P.two_A = false;
        
        
        if (P.two_B==true){
            P.five_B = false;
            P.four_B = false;
            P.three_B = false;
            P.three_A = false;
            P.two_A = false;
        }
        if (P.five_B == true){
            P.four_B = false;
            P.two_B = false;
            P.three_B = false;
            P.three_A = false;
            P.two_A = false;
        }
        
        EA E;
        
        /////// TRAINING ///////
        P.train_set();      //set train bool to true depending on combo selected - set test bools to false
        assert(P.tr_2==true || P.tr_3==true || P.tr_4==true || P.tr_5==true);
        
        P.train();          //set A and P force ranges - set bools
        assert(P.sensor_NOISE==false && P.actuator_NOISE==false);
        assert(P.te_A==false && P.te_B==false);
        
        E.pP = &P;
        E.Run_Program();
        
        
        /////// TESTING ///////
        P.test_set();           //set test bool to true - set train bools to false
        assert(P.te_A==true || P.te_B==true);
        assert(P.tr_2==false && P.tr_3==false && P.tr_4==false && P.tr_5==false);
        
        P.test();
        //E.pP = &P;
        assert(P.A_f_max_bound==0 && P.rand_antagonist==false);
        
        E.Run_Test_Program();
        test_fit << E.test_pro_pol.at(0).P_fitness << endl;
        
        cout << "BEST POLICY Test PRO-FITNESS" << "\t" << E.test_pro_pol.at(0).P_fitness << endl;
        
        
        // SR RUN TIME
        t3 = clock();
        float df ((float)t3-(float)t1);
        float sec = df /CLOCKS_PER_SEC;
        cout << "SR time\t" << sec << endl;
        
    }//SR LOOP
    
    // TOTAL PROGRAM RUN TIME
    t2 = clock();
    float diff ((float)t2-(float)t1);
    float seconds = diff / CLOCKS_PER_SEC;
    cout << "Total Run Time" << "\t" << seconds << endl;
    
    // INCLUDE TOTAL RUN TIME IN PARAMETER FILE
    test_para.open("testing_parameters.txt", fstream::app);
    test_para << "run time" << "\t" << seconds << endl;
    
    //P.closeGraphs();
    test_fit.close();
    P_fit.close();
    SR.close();
    SR_test.close();
    
    cout << "END PROGRAM" << endl;
}
