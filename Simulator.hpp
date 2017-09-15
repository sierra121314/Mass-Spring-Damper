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


//------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------
//Runs the entire simulation process
void Simulator::Simulate(Policy* pPo)
{
    //pPo->weights;
    
    //intialize starting stuff
    pPo->x = pP->start_x-pP->displace; //starting position minus any displacement
    pPo->x_dot = pP->start_x_dot;
    pPo->x_dd = pP->start_x_dd;
    pPo->P_force = pP->start_P_force;

    neural_network NN;

    NN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs); //3 input, 5 hidden, 1 output
    
    NN.set_in_min_max(pP->x_min_bound, pP->x_max_bound); //displacement
    NN.set_in_min_max(pP->x_dot_min_bound,pP->x_dot_max_bound); //velocity
    //NN.set_in_min_max(pP->x_dd_min_bound,pP->x_dd_max_bound); //acceleration
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
    
    //CLEAR x, xdot, xdd history vector
    pPo->x_history.clear();
    pPo->x_dot_history.clear();
    pPo->x_dd_history.clear();
    pPo->P_force_history.clear();
    //cout << pPo->x_history.size() << endl;
    
    for (int i = 0; i < pP->total_time; i++) { // has to run long enough to change directions
        
        //give state vector to give to NN in order to update P_force
        vector<double> state;
        state.push_back(pPo->x);
        state.push_back(pPo->x_dot);
        state.push_back(pPo->x_dd);
        //cout << pPo->x << "\t" << pPo->x_dot << "\t" << pPo->x_dd << endl;
        
        NN.set_vector_input(state);
        
        NN.execute();
        
        cout << NN.scInput() << endl;
        pPo->P_force = NN.get_output(0);
        cout << pPo->P_force << endl;
        cout << "i " << i << endl;
        assert(pPo->P_force >= pP->f_min_bound - 0.5 && pPo->P_force <= pP->f_max_bound + 0.5); //make sure matches NN output
        
        // UPDATE POSITION, VELOCITY, ACCELERATION //
        
        pPo->x_dd = (1/(pPo->m))*((-pPo->b*pPo->x_dot) - (pPo->k*(pPo->x-pP->start_x)) + pPo->P_force - pPo->mu);
        
        pPo->x_dot = pPo->x_dot + pPo->x_dd*pPo->dt;
        pPo->x = pPo->x + pPo->x_dot*pPo->dt;
        
        //cout << pPo->x_dd << "\t" << pPo->x_dot << "\t" << pPo->x << endl;

        //calculate fitness
        double ss_penalty = 0;
        if (pPo->x<1.05*(1 + pP->start_x) || pPo->x>.95*(1 + pP->start_x)) {
            ss_penalty = 1; //want closest to 0 displacement and penalize for not being at Steady state
        }
        
        double F_dist = (abs(2 + pP->start_x - pPo->x)); //want closest to 0 displacement
        
        pPo->fitness += pP->w1*F_dist + pP->w2*ss_penalty;
        //cout << "F_dist" << "\t" << endl;
        pPo->x_history.push_back(pPo->x);
        pPo->x_dot_history.push_back(pPo->x_dot);
        pPo->x_dd_history.push_back(pPo->x_dd);
        pPo->P_force_history.push_back(pPo->P_force);
    }
    
    //cout << "end of simulator loop" << endl;
    
    
}


#endif /* Simulator_hpp */
