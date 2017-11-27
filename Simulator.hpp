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
#include <cstdlib>
#include <cmath>
#include <limits>
#define PI 3.1415926535897


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
    Parameters* aP;
    
    void Simulate(Policy* pPo, Policy* aPo);
    void MSD_initStates(Policy* pPo, Policy* aPo);
    void Pendulum_initStates(Policy* pPo, Policy* aPo);
    double generateGaussianNoise();
    double generateActuatorNoise();
    double generateSensorNoise();
    void calculateFitness(Policy* pPo, Policy* aPo);
    double Fh = 60; //Frequency in hertz
    double noise_x_sum;
    double noise_xdot_sum;
    double xt =0;    //noise sinusoidal
    double g_xt = 0; //goal sinusoidal
    double sinusoidal=0;
    void MSD_equations(Policy* pPo, Policy* aPo);
    void Pendulum_equations(Policy* pPo, Policy* aPo);
    
private:
};



//------------------------------------------------------------------------------------------------------------
void Simulator::MSD_initStates(Policy* pPo, Policy* aPo){
    noise_x_sum = 0;
    noise_xdot_sum = 0;
    
    //intialize starting stuff
    pPo->x = pP->start_x-pP->displace; //starting position minus any displacement
    pPo->x_dot = pP->start_x_dot;
    pPo->x_dd = pP->start_x_dd;
    
    pP->P_force = pP->start_P_force;
    pP->A_force = pP->start_A_force;

}

void Simulator::Pendulum_initStates(Policy *pPo, Policy *aPo){
    noise_x_sum = 0;
    noise_xdot_sum = 0;
    //theta = start_theta - displace;
    //theta_dot = start_theta_dot;
    //theta_dd = start_theta_dd;
    
    pP->P_force = pP->start_P_force;
    pP->A_force = pP->start_A_force;
}

double Simulator::generateGaussianNoise() {
    const double epsilon = numeric_limits<double>::min();
    const double two_pi = 2.0*PI;
    double mu = 0;
    //double mu = (((double)rand() / RAND_MAX) - 0.5) * 2;
    double sigma = 0.4; //0.4
    //double sigma = (((double)rand() / RAND_MAX) - 0.5) * 2;
    double n = 0;
    //cout << mu << "," << sigma << endl;
    double z0, z1;
    double u1=0, u2=0;
    
    do{
        //cout << u1 << "," u2 << endl;
        u1 = ((double)rand() / RAND_MAX);
        u2 = ((double)rand() / RAND_MAX);
        //cout << u1 << "," << u2 << endl;
    } while (u1 <= epsilon);
    z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
    z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
    //cout << z0 << endl;
    n = z0 * sigma + mu;
    while (n>1 || n<-1){
        //cout << "new error loop" << endl;
        double z0, z1;
        double u1=0, u2=0;
    
        do{
            //cout << u1 << "," u2 << endl;
            u1 = ((double)rand() / RAND_MAX);
            u2 = ((double)rand() / RAND_MAX);
            //cout << u1 << "," << u2 << endl;
        } while (u1 <= epsilon);
        z0 = sqrt(-2.0 * log(u1)) * cos(two_pi * u2);
        z1 = sqrt(-2.0 * log(u1)) * sin(two_pi * u2);
        //cout << z0 << endl;
        n = (z0 * sigma + mu)/10;
    }
    return n;
}

void Simulator::MSD_equations(Policy* pPo, Policy* aPo){
    if (pP->rand_antagonist==true){
        pP->A_force=0;
    }
    pPo->x_dd = (1/(pP->m))*((-pP->b*pPo->x_dot) - (pP->k*(pPo->x-pP->start_x)) + pP->P_force + pP->A_force - pP->mu);
    
    pPo->x_dot = pPo->x_dot + pPo->x_dd*pP->dt;
    pPo->x = pPo->x + pPo->x_dot*pP->dt;
}

void Simulator::Pendulum_equations(Policy *pPo, Policy *aPo){
    //theta_dd = -g*sin(theta) / (L); //rad/s^2   // define theta_dd with t variable
    //thetat_dd to theta_dot
    //theta_dot = theta_dot + theta_dd*dt;
    //theta_dot to theta
    //theta = theta + theta_dot*dt;
    //theta to xy
    //Px = L*cos(theta);
    //Py = L*sin(theta);
}

void Simulator::calculateFitness(Policy* pPo, Policy* aPo){
    double ss_penalty = 0;
    /*
     if (pPo->x<1.05*(1 + pP->start_x) || pPo->x>.95*(1 + pP->start_x)) {
     ss_penalty = 1; //want closest to 0 displacement and penalize for not being at Steady state
     }
     */
    if (pP->sinusoidal_goal==true){  // SINUSOIDAL GOAL //
        g_xt = pPo->x+g_xt+pP->dt;
        pP->goal_x = pP->start_x + pP->A_g*sin(PI/16*(g_xt+pP->dt)+pP->g_phase);
        double F_dist = (abs(pP->goal_x - pPo->x));
        pPo->P_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
        aPo->A_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
    }
    else if (pP->multi_var==true){
        pP->goal_x = pP->goal_x;//goal.at(k) from list
        //cout << pP->goal_x << endl;
        double F_dist = (abs(pP->goal_x + pP->start_x - pPo->x)); //2 + resting position
        pPo->P_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
        aPo->A_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
    }
    else {
        pP->goal_x = 2;
        double F_dist = (abs(pP->goal_x + pP->start_x - pPo->x)); //2 + resting position
        pPo->P_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
        aPo->A_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
    }

}


//-------------------------------------------------------------------------
//Runs the entire simulation process
void Simulator::Simulate(Policy* pPo, Policy* aPo)
{
    //NOISE GRAPH
    fstream nsensor;
    fstream nactuator;
    nsensor.open("ave_sensor_noise.txt", fstream::app);
    nactuator.open("ave_actuator_noise.txt", fstream::app);
    ofstream tstep_sensor;
    tstep_sensor.open("tstep_sensor.txt", ofstream::out | ofstream::trunc);
    ofstream tstep_actuator;
    tstep_actuator.open("tstep_actuator.txt", ofstream::out | ofstream::trunc);
    
    fstream rand_start;
    rand_start.open("random_starting_variables.txt", fstream::app);
    
    //STARTING POSITIONS
    MSD_initStates(pPo, aPo);
    
    //LOGGING START POSITIONS
    rand_start << pP->m << "\t" << pP->b << "\t" << pP->k << "\t" << pP->mu << "\t" << pP->start_x << endl;
    
    // PROTAGONIST //
    neural_network NN;
    NN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs); //2 input, 10 hidden, 1 output
    NN.set_in_min_max(pP->x_min_bound, pP->x_max_bound);        //displacement
    NN.set_in_min_max(pP->x_dot_min_bound,pP->x_dot_max_bound); //velocity
    NN.set_out_min_max(pP->P_f_min_bound,pP->P_f_max_bound); // max forces
    NN.set_weights(pPo->P_weights, true);
    
    // ANTAGONIST //
    neural_network NNa;
    if (pP->rand_antagonist ==true) {
        
    }
    else {
        // ANTAGONIST //
        
        NNa.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs);
        NNa.set_in_min_max(pP->x_min_bound, pP->x_max_bound);        //displacement
        NNa.set_in_min_max(pP->x_dot_min_bound,pP->x_dot_max_bound);  //velocity
        NNa.set_out_min_max(pP->A_f_min_bound,pP->A_f_max_bound); // max forces
        NNa.set_weights(aPo->A_weights, true);

    }


    //CLEAR x, xdot, xdd history vector
    pPo->x_history.clear();
    pPo->x_dot_history.clear();
    pPo->x_dd_history.clear();
    pPo->P_force_history.clear();
    aPo->A_force_history.clear();
    //cout << pPo->x_history.size() << endl;
    
    for (int i = 0; i < pP->total_time; i++) { // has to run long enough to change directions
        
        //give state vector to give to NN in order to update P_force
        vector<double> state;
        vector<double> noise;
        noise.push_back(0); // x sensor
        noise.push_back(0); // x actuator
        noise.push_back(0); // xdot sensor
        noise.push_back(0); // xdot actuator
        if (pP->sensor_NOISE == true) {
            double r = generateGaussianNoise();
            assert(r<=1 && r>=-1);
            
            if (pP->sinusoidal_noise==true){
                xt = xt+pP->dt;
                sinusoidal = pP->As*sin(PI/8*(xt+pP->dt)+pP->phase);
            }
            
            double yt = r + sinusoidal;
            noise.at(0)= yt;
            
            double rr = generateGaussianNoise();
            assert(rr<=1 && rr>=-1);
            //sinusoidal = sin(2*PI*(xt+pP->dt)+pP->phase);
            yt = rr + sinusoidal;
            noise.at(2)= yt;
            //cout << "r=" << r << "\t" << "rr=" << rr << endl;
        }
        if (pP->actuator_NOISE == true) {
            double r = generateGaussianNoise();
            assert(r<=1 && r>=-1);
            
            if (pP->sinusoidal_noise==true){
                xt = xt+pP->dt;
                sinusoidal = pP->As*sin(PI/8*(xt+pP->dt)+pP->phase);
            }
            double yt = r + sinusoidal;
            noise.at(1)= yt;
            
            double rr = generateGaussianNoise();
            assert(rr<=1 && rr>=-1);
            yt = rr + sinusoidal;
            noise.at(3)= yt;
        }
        
        noise.at(0) = pP->sn*noise.at(0); //sensor noise for position
        noise.at(1) = pP->an*noise.at(1); //actuator noise for position
        noise.at(2) = pP->sn*(1/10)*noise.at(2); //sensor noise for velocity
        noise.at(3) = pP->an*noise.at(3); //actuator noise for velocity
        state.push_back(pPo->x+noise.at(0)+noise.at(1));
        state.push_back(pPo->x_dot+noise.at(2)+noise.at(3));
        
        NN.set_vector_input(state);
        NN.execute();
        pP->P_force = NN.get_output(0);
        assert(pP->P_force >= pP->P_f_min_bound - 0.5 && pP->P_force <= pP->P_f_max_bound + 0.5); //make sure matches NN output
        if (pP->rand_antagonist==true){
            
        }
        else {
            NNa.set_vector_input(state);
            NNa.execute();
            pP->A_force = NNa.get_output(0);
            assert(pP->A_force >= pP->A_f_min_bound - 0.5 && pP->A_force <= pP->A_f_max_bound + 0.5);
            
        }
        
        // UPDATE POSITION, VELOCITY, ACCELERATION //
        if (pP->rand_start_ts == true){
            pP->random_variables();
        }
        
        MSD_equations(pPo, aPo);
        //Pendulum_equations(pPo, aPo);
       
        // CALCULATE FITNESS //
        calculateFitness(pPo, aPo);
        
        pPo->x_history.push_back(pPo->x);
        pPo->x_dot_history.push_back(pPo->x_dot);
        pPo->x_dd_history.push_back(pPo->x_dd);
        pPo->P_force_history.push_back(pP->P_force);
        aPo->A_force_history.push_back(pP->A_force);
        
        noise_x_sum += noise.at(0)+noise.at(1);
        noise_xdot_sum += noise.at(2)+noise.at(3);
        tstep_sensor << noise.at(0)+noise.at(1) << "\t";
        tstep_actuator << noise.at(2)+noise.at(3) << "\t";
    }
    
    nsensor << (noise_x_sum/(2*pP->total_time)) << "\t";
    nactuator << (noise_xdot_sum/(2*pP->total_time)) << "\t";
    
    tstep_sensor.close();
    tstep_actuator.close();
}


#endif /* Simulator_hpp */
