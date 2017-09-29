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
    E.pP = &P; //what is this?
    //E.aP = &A;
    E.Run_Program();
    
    /*
    if (P.train_and_test == true){
        //Parameters P;
        P.train();
        EA E;
        E.pP = &P;
        E.Run_Program();
        
        cout << "test" << endl;
        P.test();
        E.Run_Program(); //can't do this - population assert 
        
     
    }
    else { //run however parameters are set up
        EA E;
        E.pP = &P;
        //E.aP = &A;
        E.Run_Program();
    }
    */
    cout << "END PROGRAM" << endl;
}
