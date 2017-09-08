//
//  Simulator.hpp
//  Pro-Ant_EA_project
//
//  Created by Sierra Gonzales on 8/21/17.
//  Copyright © 2017 Pro-Ant_EA_project. All rights reserved.
//

#ifndef Simulator_hpp
#define Simulator_hpp

#include "LY_NN.h"
#include <stdio.h>

using namespace std;


class Simulator
{
    friend class EA;
    friend class Parameters;
    friend class Policy;
    friend class neural_network;
    
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
    pPo->x = pP->start_x-pP->displace;
    pPo->x_dot = pP->start_x_dot;
    pPo->x_dd = pP->start_x_dd;
    pPo->P_force = pP->start_P_force;
    //clear x, xdot, xdd history vector
    
    neural_network NN;

    NN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs); //3 input, 5 hidden, 1 output
    
    NN.set_in_min_max(pP->x_min_bound, pP->x_max_bound); //displacement
    NN.set_in_min_max(pP->x_dot_min_bound,pP->x_dot_max_bound); //velocity
    NN.set_in_min_max(pP->x_dd_min_bound,pP->x_dd_max_bound); //acceleration
    NN.set_out_min_max(pP->f_min_bound,pP->f_max_bound); // max forces
    NN.set_weights(pPo->weights, true);
    //cout << pPo->weights.size() << endl;
    //cout << endl;
    //cout << pP->num_weights << endl;
    /*
    for (int w=0; w<pP->num_weights; w++)
    {
        cout << pPo->weights.at(w) << "\t";
    }
    cout << endl;
     */
    pPo->x_history.clear();
    //cout << pPo->x_history.size() << endl;
    
    for (int i = 0; i < 1000; i++) { // has to run long enough to change directions
        
        //give state vector to give to NN in order to update P_force
        vector<double> state;
        state.push_back(pPo->x);
        state.push_back(pPo->x_dot);
        state.push_back(pPo->x_dd);
        
        NN.set_vector_input(state);
        NN.execute();
        pPo->P_force = NN.get_output(0);
        assert(pPo->P_force >= pP->f_min_bound && pPo->P_force <= pP->f_max_bound); //make sure matches NN output
        
        // UPDATE POSITION, VELOCITY, ACCELERATION //
        //pPo->x_dd = (1/(pPo->m))*((-pPo->b*pPo->x_dot) - (pPo->k*(pPo->x-pP->start_x)) + pPo->P_force - pPo->mu); // (x - start_x in order to not have ICs //
        
        
        
        
        // TEST DISPLACEMENT //
        pPo->x_dd = (1/(pPo->m))*((-pPo->b*pPo->x_dot) - (pPo->k*(pPo->x-pP->start_x)) + pPo->P_force - pPo->mu);
        
        pPo->x_dot = pPo->x_dot + pPo->x_dd*pPo->dt;
        pPo->x = pPo->x + pPo->x_dot*pPo->dt;
        
        //cout << pPo->x_dd << "\t" << pPo->x_dot << "\t" << pPo->x << endl;
        //NEED:push each variable into its own vector instead of outputting into text file (history file)
        
        //calculate fitness
        double F_dist = (abs(pP->start_x - pPo->x)); //want closest to 0 displacement
        pPo->fitness += F_dist;
        //cout << "F_dist" << "\t" << F_dist << endl;
        pPo->x_history.push_back(pPo->x);
    }
    
    //cout << "end of simulator loop" << endl;
    
    
    
    
}


#endif /* Simulator_hpp */
