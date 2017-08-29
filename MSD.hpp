//
//  MSD_main.hpp
//  Project
//
//  Created by Sierra Gonzales on 8/27/17.
//  Copyright © 2017 Sierra Gonzales. All rights reserved.
//

#ifndef MSD_hpp
#define MSD_hpp

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <vector>
#include <iostream>


namespace MassSpringDamper {  // Mass Spring Damper
    
    // global constants
    const double g = 9.81; // gravity - m/s^2
    const double dt = 0.01; // time step - s
    const double mu = 0; //friction
    
    // mass structure
    struct MassState {
        // mass of the cart - constant
        const int mass_c;
        // constrained to x axis
        double x; // position
        double x_dot; // velocity
        double x_dd; // acceleration
    };
    
    
    class MSD {
    private:
        /// variables
        // all actions for log
        std::vector <double> force_history;
        // all fitness' for log
        std::vector <double> fitness_history;
        // all states.  last element is most current
        std::vector <MassState> msd_state;
        // applied force - action
        double force;
        // last determined fitness
        double fitness;
        /// functions
        // determine fitness
        double determine_reward();
    public:
        MSD();
        // static variables
        double mass_msd; // mass of mass
        double b; // inital damper
        double k; // spring constant
        double x_init;
        /// functions
        // return last state
        std::vector <double> give_state();
        // get action then cycle
        void get_action(std::vector <double>);
        // return last reward
        std::vector <double> give_reward();
        // calculates next state given previous and an action
        void cycle();
        // log all state history to file
        void export_all_states();
    };
    
    MSD::MSD()
    {
        MassState initial;
        // - settings begin -
        mass_msd = 5; // kg
        k = 1; //
        b = 1; //
        // - settings end -
        
        // do not modify - setup inital state
        initial.x = x_init;
        initial.x_dot = 0; // m/s
        initial.x_dd = 0;  // m/s^2
        force = 0; // N*m
        msd_state.push_back(initial);
        force_history.push_back(force);
        fitness_history.push_back(determine_reward());
    }
    
    // calculates the next state given a previously set action with a timestep of 'dt'
    // note: aciton must have been set. use 'get_action'
    void MSD::cycle() {
        msd_state nextState;
        // - calculations -
        // torque to theta dd
        nextState.x_dd = (1/m)*(-b*msd_state.at(msd_state.size()-1).x_dot + k*msd_state.at(msd_state.size()-1).x +force - mu;
        // x_dd to x_dot
        nextState.x_dot = msd_state.at(msd_state.size() - 1).x_dot + nextState.x_dd*dt;
        // theta_dot to theta
        nextState.x = msd_state.at(msd_state.size() - 1).x + nextState.x_dot*dt;
        //// - END CALCULATIONS - ////
        // keep theta between 0 and 2*PI
        assert (nextState.x < 0.0);
        
#ifdef CB_CONSULE
        std::cout << nextState.x << "," << nextState.x_dot << "," \
        << nextState.x_dd << "," \
        << std::endl;
#endif
        msd_state.push_back(nextState); // save new state
        force_history.push_back(force); // save torq for log
        fitness = determine_reward(); // determine fitness
        fitness_history.push_back(fitness);
    }
    
    // determine reward based on current state
    // max range for each fitness is 0 to 1 times its set weight
    double MSD::determine_reward() {
        
        // fitness weights
        // ---------------------
        double tp_weight = 1.0;  	// position
        double tv_weight = 30.0;  	// velocity
        double ch_weight = 100000.0;  // below vertical axis
        double tu_weight = 0.0;  	// force used penalty - not normalized
        // ---------------------
        
        // x position
        double fitness_1 = abs((M_PI/2 - pend.at(pend.size()-1).theta)*tp_weight)/2*M_PI;
        // x velocity
        double fitness_2 = abs((pend.at(pend.size()-1).theta_dot)*tv_weight);
        if (fitness_2 > 10) fitness_2 = 10;
        fitness_2 = fitness_2/10;
        // below vertical axis (change to pass)
        double fitness_ch = 0.0;
        if (msd_state.at(msd_state.size()-1).x > 0) fitness_ch = ch_weight;
        // effort used
        double fitness_tu = tu_weight * force;
        
        double total_fitness;
        total_fitness = fitness_1 + fitness_2 + fitness_ch;
        return total_fitness;
    }
    
    
    // --------------------------------------
    // return current state
    // - x, x_dot
    std::vector <double> MSD::give_state() {
        std::vector <double> temp_state;
        temp_state.push_back(msd_state.at(msd_state.size()-1).x);
        temp_state.push_back(msd_state.at(msd_state.size()-1).x_dot);
        return temp_state;
    }
    
    // get action from controller
    // vector with force in first element
    void MSD::get_action(std::vector <double> in_action) {
        torq = in_action.at(0);
        cycle();
    }
    
    // return reward for current state
    std::vector <double> Pendulum::give_reward() {
        std::vector <double> total_fitness;
        total_fitness.push_back(fitness);
        
        return total_fitness;
    }
    // --------------------------------------
    
    // export all state history to file "pend_state_log.csv" for review
    void MSD::::export_all_states() {
        std::ofstream fout;
        fout.open("msd_state_log.csv", std::ofstream::out | std::ofstream::trunc);
        fout << "torq, x, y, theta, theta_dot, theta_dd, fitness" << "\n";
        for (std::size_t i=0; i<pend.size(); ++i) {
            fout << force_history.at(i) << ", "   << msd_state.at(i).x* << \
            ", " << msd_state.at(i).x_dot << ", " << msd_state.at(i).x_dd << \
            ", " << fitness_history.at(i) << "\n";
        }	
        fout.close();
    }
    
}

#endif /* MSD_hpp */
