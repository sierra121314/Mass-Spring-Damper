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



int main()
{
    srand(time(NULL));
    Parameters P;
    EA E;
    
    //TRAINING MODES
    P.train_and_test = true;
    P.tr_1 = true;
    P.tr_2 = false;
    P.te_1 = false;
    P.te_2 = false;
    
    if (P.train_and_test == true){
        P.train();
        E.pP = &P;
        E.Run_Program();
        if(P.te_1==true | P.te_2==true){
            P.test();
            E.pP = &P;
            E.Run_Program();
        }
    }
    
    else {
        P.P_f_min_bound = -5;
        P.P_f_max_bound = 5;
        P.A_f_min_bound = -2;
        P.A_f_max_bound = 2;
        
        E.pP = &P;
        E.Run_Program();
        
    }
    
    
    cout << "END PROGRAM" << endl;
}
