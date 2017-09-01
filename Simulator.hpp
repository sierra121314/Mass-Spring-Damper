//
//  Simulator.hpp
//  Pro-Ant_EA_project
//
//  Created by Sierra Gonzales on 8/21/17.
//  Copyright © 2017 Pro-Ant_EA_project. All rights reserved.
//

#ifndef Simulator_hpp
#define Simulator_hpp

#include <stdio.h>

using namespace std;


class Simulator
{
    friend class EA;
    friend class Parameters;
    friend class Policy;
    
protected:
    
    
public:
    Parameters* pP;
    
    void Simulate(Policy* pPo);
    
    
private:
};


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------
//Runs the entire simulation process
void Simulator::Simulate(Policy* pPo)
{
    //pPo->weights;
    
    //intialize starting stuff
    int start_x = 10;
    pPo->x = start_x;
    int start_x_dot = 0;
    pPo->x_dot = start_x_dot;
    int start_x_dd = 0;
    pPo->x_dd = start_x_dd;
    int start_P_force = 0;
    pPo->P_force = start_P_force;
    //clear x, xdot, xdd history vector
    
    for (int i = 0; i < 500; i++) {
        //update pos, vel, acc
        
        pPo->x_dd = (1/(pPo->m))*((-pPo->b*pPo->x_dot) - (pPo->k*(pPo->x - start_x)) + pPo->P_force - pPo->mu); // (x - start_x in order to not have ICs //
        pPo->x_dot = pPo->x_dot + pPo->x_dd*pPo->dt;
        pPo->x = pPo->x + pPo->x_dot*pPo->dt;
        
        cout << pPo->x_dd << "\t" << pPo->x_dot << "\t" << pPo->x << endl;
    }
    
    
    
    
}


#endif /* Simulator_hpp */