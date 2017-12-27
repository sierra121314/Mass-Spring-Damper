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
    //Parameters* aP;
    
    void Simulate(Policy* pPo, Policy* aPo);
    vector<double> set_state(vector<double> state, vector<double> noise, Policy* pPo, Policy* aPo);
    void MSD_initStates(Policy* pPo, Policy* aPo);
    void Pendulum_initStates(Policy* pPo, Policy* aPo);
    
    void calculateFitness(Policy* pPo, Policy* aPo);
    double Fh = 60; //Frequency in hertz
    double g_xt = 0; //goal sinusoidal
    double sinusoidal=0;
    void MSD_equations(Policy* pPo, Policy* aPo);
    void Pendulum_equations(Policy* pPo, Policy* aPo);
    
    // PRIMARY //
    double set_P_force(neural_network NN, vector<double> state);
    neural_network set_P_NN(neural_network NN, Policy* pPo);
    
    // ANTAGONIST //
    void set_A_ICs(Policy* pPo, Policy* aPo); //2nd Antagonist
    double set_Ant_force(neural_network NNa, vector<double> state);
    neural_network set_A_NN(neural_network NNa, Policy* aPo);
    
    // NOISE //
    double generateGaussianNoise();
    double generateActuatorNoise();
    double generateSensorNoise();
    double noise_x_sum;
    double noise_xdot_sum;
    double xt =0;    //noise sinusoidal
    vector<double> noise_init(vector<double> noise);
    vector<double> set_sensor_actuator_noise(vector<double> noise);
    double x_sensor_noise();
    double xdot_sensor_noise();
    double x_actuator_noise();
    double xdot_actuator_noise();
    
    // HISTORY //
    void history(Policy* pPo, Policy* aPo);
    void clear_history(Policy* pPo, Policy* aPo);
    
private:
};



//------------------------------------------------------------------------------------------------------------
void Simulator::set_A_ICs(Policy* pPo, Policy* aPo){
    pP->goal_x = aPo->A_ICs.at(0);
    pP->start_x = aPo->A_ICs.at(1);
    pP->start_x_dot = aPo->A_ICs.at(2);
}

void Simulator::MSD_initStates(Policy* pPo, Policy* aPo){
    noise_x_sum = 0;
    noise_xdot_sum = 0;
    
    //intialize starting stuff
    if (pP->tr_5==true){
        set_A_ICs(pPo, aPo);
    }
    
    pPo->x = pP->start_x-pP->displace; //starting position minus any displacement
    pPo->x_dot = pP->start_x_dot;
    pPo->x_dd = pP->start_x_dd;
    pP->P_force = pP->start_P_force;
    pP->A_force = pP->start_A_force;
    
}


neural_network Simulator::set_P_NN(neural_network NN, Policy* pPo){
    NN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs); //2 input, 10 hidden, 1 output
    NN.set_in_min_max(pP->x_min_bound, pP->x_max_bound);        //displacement
    NN.set_in_min_max(pP->x_dot_min_bound,pP->x_dot_max_bound); //velocity
    NN.set_out_min_max(pP->P_f_min_bound,pP->P_f_max_bound); // max forces
    NN.set_weights(pPo->P_weights, true);
    return NN;
}

neural_network Simulator::set_A_NN(neural_network NNa, Policy* aPo){
    NNa.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs);
    NNa.set_in_min_max(pP->x_min_bound, pP->x_max_bound);        //displacement
    NNa.set_in_min_max(pP->x_dot_min_bound,pP->x_dot_max_bound);  //velocity
    NNa.set_out_min_max(pP->A_f_min_bound,pP->A_f_max_bound); // max forces
    NNa.set_weights(aPo->A_weights, true);
    return NNa;
}

void Simulator::clear_history(Policy* pPo, Policy* aPo){
    pPo->x_history.clear();
    pPo->x_dot_history.clear();
    pPo->x_dd_history.clear();
    pPo->P_force_history.clear();
    aPo->A_force_history.clear();
}


vector<double> Simulator::noise_init(vector<double> noise){
    noise.push_back(0); // x sensor
    noise.push_back(0); // x actuator
    noise.push_back(0); // xdot sensor
    noise.push_back(0); // xdot actuator
    return noise;
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

double Simulator::x_sensor_noise(){
    double r = generateGaussianNoise();
    assert(r<=1 && r>=-1);
    
    if (pP->sinusoidal_noise==true){
        xt = xt+pP->dt;
        sinusoidal = pP->As*sin(PI/8*(xt+pP->dt)+pP->phase);
    }
    double yt = r + sinusoidal;
    
    return yt;
}
double Simulator::xdot_sensor_noise(){
    double rr = generateGaussianNoise();
    assert(rr<=1 && rr>=-1);
    if (pP->sinusoidal_noise==true){
        xt = xt+pP->dt;
        sinusoidal = pP->As*sin(PI/8*(xt+pP->dt)+pP->phase);
    }
    double yt = rr + sinusoidal;
    
    return yt;
}

double Simulator::x_actuator_noise(){
    double r = generateGaussianNoise();
    assert(r<=1 && r>=-1);
    
    if (pP->sinusoidal_noise==true){
        xt = xt+pP->dt;
        sinusoidal = pP->As*sin(PI/8*(xt+pP->dt)+pP->phase);
    }
    double yt = r + sinusoidal;
    return yt;
}

double Simulator::xdot_actuator_noise(){
    double rr = generateGaussianNoise();
    assert(rr<=1 && rr>=-1);
    if (pP->sinusoidal_noise==true){
        xt = xt+pP->dt;
        sinusoidal = pP->As*sin(PI/8*(xt+pP->dt)+pP->phase);
    }
    double yt = rr + sinusoidal;
    return yt;
}

vector<double> Simulator::set_sensor_actuator_noise(vector<double> noise){
    if (pP->sensor_NOISE == true) {
        noise.at(0)= x_sensor_noise();
        noise.at(2)= xdot_sensor_noise();
    }
    if (pP->actuator_NOISE == true) {
        noise.at(1)= x_actuator_noise();
        noise.at(3)= xdot_actuator_noise();
    }
    
    noise.at(0) = pP->sn*noise.at(0); //sensor noise for position
    noise.at(1) = pP->an*noise.at(1); //actuator noise for position
    noise.at(2) = pP->sn*(1/10)*noise.at(2); //sensor noise for velocity
    noise.at(3) = pP->an*noise.at(3); //actuator noise for velocity
    return noise;
}

vector<double> Simulator::set_state(vector<double> state, vector<double> noise, Policy* pPo, Policy* aPo){
    state.push_back(pPo->x+noise.at(0)+noise.at(1));
    state.push_back(pPo->x_dot+noise.at(2)+noise.at(3));
    return state;
}

double Simulator::set_P_force(neural_network NN, vector<double> state){
    NN.set_vector_input(state);
    NN.execute();
    pP->P_force = NN.get_output(0);
    assert(pP->P_force >= pP->P_f_min_bound - 0.5 && pP->P_force <= pP->P_f_max_bound + 0.5); //make sure matches NN output
    return pP->P_force;
}

double Simulator::set_Ant_force(neural_network NNa, vector<double> state){
    if (pP->tr_3==true) {
        NNa.set_vector_input(state);
        NNa.execute();
        pP->A_force = NNa.get_output(0);
        assert(pP->A_force >= pP->A_f_min_bound - 0.5 && pP->A_force <= pP->A_f_max_bound + 0.5);
    }
    else if (pP->rand_antagonist==true){
        pP->A_force = 0;
    }
    else{
        pP->A_force = 0;
    }
    
    return pP->A_force;
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


void Simulator::MSD_equations(Policy* pPo, Policy* aPo){

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

void Simulator::history(Policy* pPo, Policy* aPo){
    pPo->x_history.push_back(pPo->x);
    pPo->x_dot_history.push_back(pPo->x_dot);
    pPo->x_dd_history.push_back(pPo->x_dd);
    pPo->P_force_history.push_back(pP->P_force);
    aPo->A_force_history.push_back(pP->A_force);
    
}


//-------------------------------------------------------------------------
// RUNS ENTIRE SIMULATION PROCESS //
void Simulator::Simulate(Policy* pPo, Policy* aPo){
    
    pPo->P_fit_swap = 0;
    aPo->A_fit_swap = 0;
    
    // NOISE GRAPH //
    fstream nsensor, nactuator, tstep_sensor, tstep_actuator;
    nsensor.open("ave_sensor_noise.txt", fstream::app);
    nactuator.open("ave_actuator_noise.txt", fstream::app);
    tstep_sensor.open("tstep_sensor.txt", ofstream::out | ofstream::trunc);
    tstep_actuator.open("tstep_actuator.txt", ofstream::out | ofstream::trunc);
    
    // STARTING POSITIONS //
    MSD_initStates(pPo, aPo);
    
    // PROTAGONIST //
    neural_network NN;
    NN = set_P_NN(NN, pPo);
    
    // ANTAGONIST //
    neural_network NNa;
    if (pP->tr_3==true){
        NNa = set_A_NN(NNa, aPo);
    }

    //CLEAR x, xdot, xdd history vector
    clear_history(pPo, aPo);
    
    for (int i = 0; i < pP->total_time; i++) { // has to run long enough to change directions
        //give state vector to give to NN in order to update P_force
        vector<double> state;
        vector<double> noise;
        noise = noise_init(noise); //set sensor and actuator noise to zero
        noise = set_sensor_actuator_noise(noise); //if noise -> update noise

        state = set_state(state, noise, pPo, aPo); //set position and velocity with added noise
        
        /////// PRIMARY ///////
        pP->P_force = set_P_force(NN, state);
        
        /////// ANTAGONIST ///////
        pP->A_force = set_Ant_force(NNa, state);
        
        // UPDATE POSITION, VELOCITY, ACCELERATION //
        MSD_equations(pPo, aPo);
        //Pendulum_equations(pPo, aPo);
       
        // CALCULATE FITNESS //
        calculateFitness(pPo, aPo);
        
        // STORE HISTORY (STATES AND FORCES) //
        history(pPo, aPo);
        
        noise_x_sum += noise.at(0)+noise.at(1); //QUESTION: should this be absolute value?
        noise_xdot_sum += noise.at(2)+noise.at(3);
        tstep_sensor << noise_x_sum << "\t";
        tstep_actuator << noise_xdot_sum << "\t";
    }
    
    nsensor << (noise_x_sum/(2*pP->total_time)) << "\t";
    nactuator << (noise_xdot_sum/(2*pP->total_time)) << "\t";
    
    tstep_sensor.close();
    tstep_actuator.close();
}


#endif /* Simulator_hpp */
