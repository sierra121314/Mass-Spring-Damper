//
//  MSD_main.cpp
//  Project
//
//  Created by Sierra Gonzales on 8/27/17.
//  Copyright © 2017 Sierra Gonzales. All rights reserved.
//

#include "MSD.hpp"


int main()
{
    // variables
    MSD::MassState msd_state;
    
    for (std::size_t i=0; i<10; ++i) {
        // calculate all of the states of the pendulum along the curve
        msd_state.cycle();
    }
    
    msd_state.export_all_states();
    
    // initialize cart weight
    //Cart cart; // Object call cart of type cart
    //cart.x = 0;
    
    return 0;
}