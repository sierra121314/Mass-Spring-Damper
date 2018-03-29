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
    void MSD_initStates(Policy* pPo, Policy* aPo);
    void Pend_initStates(Policy* pPo, Policy* aPo);
    void Pendulum_initStates(Policy* pPo, Policy* aPo);
    double generateGaussianNoise();
    double generateActuatorNoise();
    double generateSensorNoise();
    void calculateFitness(Policy* pPo, Policy* aPo, double remaining_ts);
    double Fh = 60; //Frequency in hertz
    double x_old;
    double x_dot_old;
    double x_dd_old;

    double g_xt = 0; //goal sinusoidal
    double sinusoidal=0;
    void MSD_equations(Policy* pPo, Policy* aPo);
    void Pendulum_equations(Policy* pPo, Policy* aPo);
    
    // Runge-Kutta
    double t_init = 0;
    double t;
    double k1_w;
    double k2_w;
    double k3_w;
    double k4_w;
    double k1_theta;
    double k2_theta;
    double k3_theta;
    double k4_theta;
    double theta_dd(double theta);
    double theta_dot(double theta_d);
    
    //PRIMARY
    neural_network set_P_NN(neural_network NN, Policy* pPo);
    double set_P_force(neural_network NN, vector<double> state);
    
    //2ND ANTAGONIST
    void set_A_ICs(Policy* pPo, Policy* aPo);
    void set_A_pend_ICs(Policy* pPo, Policy* aPo);
    neural_network set_A_NN(neural_network NNa, Policy* aPo);
    
    //NOISE
    double xt =0;    //noise sinusoidal
    vector<double> noise_init(vector<double> noise);
    void sum_noise(vector<double> noise);
    void ave_noise(Policy* pPo, Policy* aPo);
    void zero_noise_sum_ave();
    
    double noise_x_sum;
    double noise_xdot_sum;
    double noise_sensor_sum;
    double noise_actuator_sum;
    
    double noise_x_increm = 0;
    double noise_xdot_increm = 0;
    double noise_sensor_increm = 0;
    double noise_actuator_increm = 0;
    
    double ave_x_noise;
    double ave_xdot_noise;
    double ave_sensor_noise;
    double ave_actuator_noise;
    
    vector<double> set_sensor_actuator_noise(vector<double> noise);
    double x_sensor_noise();
    double xdot_sensor_noise();
    double x_actuator_noise();
    double xdot_actuator_noise();
    
    // HISTORY
    void history(Policy* pPo, Policy* aPo);
    void clear_history(Policy* pPo, Policy* aPo);
    void void_history(Policy* pPo, Policy* aPo);
    
private:
};
//------------------------------------------------------------------------------------------------------------
void Simulator::clear_history(Policy* pPo, Policy* aPo){
    pPo->x_history.clear();
    pPo->x_dot_history.clear();
    pPo->x_dd_history.clear();
    pPo->P_force_history.clear();
    aPo->A_force_history.clear();
    pPo->position_noise_tstep_history.clear();
    pPo->velocity_noise_tstep_history.clear();
    pPo->sensor_noise_tstep_history.clear();
    pPo->actuator_noise_tstep_history.clear();
    
    pPo->ave_position_noise_history.clear();
    pPo->ave_velocity_noise_history.clear();
    pPo->ave_sensor_noise_history.clear();
    pPo->ave_actuator_noise_history.clear();
    
    pPo->P_force_history.push_back(0);
    aPo->A_force_history.push_back(0);
}
void Simulator::history(Policy* pPo, Policy* aPo){
    pPo->x_history.push_back(pPo->x);
    pPo->x_dot_history.push_back(pPo->x_dot);
    pPo->x_dd_history.push_back(pPo->x_dd);
    pPo->P_force_history.push_back(pP->P_force);
    aPo->A_force_history.push_back(pP->A_force);
    
    //pPo->position_noise_tstep_history.push_back(noise_x_sum);
    //pPo->velocity_noise_tstep_history.push_back(noise_xdot_sum);
    //pPo->sensor_noise_tstep_history.push_back(noise_sensor_sum);
    //pPo->actuator_noise_tstep_history.push_back(noise_actuator_sum);
    pPo->position_noise_tstep_history.push_back(noise_x_increm);
    pPo->velocity_noise_tstep_history.push_back(noise_xdot_increm);
    pPo->sensor_noise_tstep_history.push_back(noise_sensor_increm);
    pPo->actuator_noise_tstep_history.push_back(noise_actuator_increm);
}
void Simulator::void_history(Policy* pPo, Policy* aPo){
    double void_hist = -10;
    pPo->x_history.push_back(void_hist);
    pPo->x_dot_history.push_back(void_hist);
    pPo->x_dd_history.push_back(void_hist);
    pPo->P_force_history.push_back(void_hist);
    aPo->A_force_history.push_back(void_hist);
    
    //pPo->position_noise_tstep_history.push_back(noise_x_sum);
    //pPo->velocity_noise_tstep_history.push_back(noise_xdot_sum);
    //pPo->sensor_noise_tstep_history.push_back(noise_sensor_sum);
    //pPo->actuator_noise_tstep_history.push_back(noise_actuator_sum);
    pPo->position_noise_tstep_history.push_back(0);
    pPo->velocity_noise_tstep_history.push_back(0);
    pPo->sensor_noise_tstep_history.push_back(0);
    pPo->actuator_noise_tstep_history.push_back(0);
}

//------------------------------------------------------------------------------------------------------------
void Simulator::set_A_ICs(Policy* pPo, Policy* aPo){
    pP->goal_x = aPo->A_ICs.at(0);
    pP->start_x = aPo->A_ICs.at(1);
    pP->start_x_dot = aPo->A_ICs.at(2);
    pP->displace = aPo->A_ICs.at(3);
}
void Simulator::set_A_pend_ICs(Policy* pPo, Policy* aPo){
    pP->start_x = aPo->A_pend_ICs.at(0);
}
void Simulator::MSD_initStates(Policy* pPo, Policy* aPo){
    //intialize starting stuff
    if (pP->rand_antagonist==true || pP->rand_ant_per_5gen == true){
        if (pP->Pend_EOM==true){
            set_A_pend_ICs(pPo, aPo);
            assert(pP->start_x == aPo->A_pend_ICs.at(0));
        }
        else {
            set_A_ICs(pPo, aPo);
            assert(pP->goal_x == aPo->A_ICs.at(0) && pP->start_x == aPo->A_ICs.at(1) && pP->start_x_dot == aPo->A_ICs.at(2) && pP->displace == aPo->A_ICs.at(3));
        }
        
    }
    else if(pP->multi_var==true){
        
    }
    else if (pP->rand_start_gen==true){
        
    }
    else{
        assert(pP->init_goal_x==pP->goal_x && pP->init_start_x==pP->start_x && pP->init_start_x_dot==pP->start_x_dot);
    }
    pPo->x = pP->start_x-pP->displace; //starting position minus any displacement
    pPo->x_dot = pP->start_x_dot;
    pPo->x_dd = pP->start_x_dd;
    pP->P_force = pP->start_P_force;
    pP->A_force = pP->start_A_force;
    

}
void Simulator::Pend_initStates(Policy* pPo, Policy* aPo){
    //intialize starting stuff
    if (pP->rand_antagonist==true || pP->rand_ant_per_5gen == true){
        set_A_pend_ICs(pPo, aPo);
        assert(pP->start_x == aPo->A_pend_ICs.at(0));
        
    }
    else if(pP->multi_var==true){
        
    }
    else if (pP->rand_start_gen==true){
        
    }
    else{
        assert(pP->init_goal_x==pP->goal_x && pP->init_start_x==pP->start_x && pP->init_start_x_dot==pP->start_x_dot);
    }
    //pP->goal_x = pP->pend_goal;
    
    pPo->x = pP->start_x; //starting position minus any displacement
    //assert(pPo->x == pP->pend_start);
    pPo->x_dot = pP->start_x_dot;
    pPo->x_dd = pP->start_x_dd;
    pP->P_force = pP->start_P_force;
    pP->A_force = pP->start_A_force;
    
    
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
neural_network Simulator::set_P_NN(neural_network NN, Policy* pPo){
    NN.setup(pP->num_inputs,pP->num_nodes,pP->num_outputs); //2 input, 10 hidden, 1 output
    NN.set_in_min_max(pP->x_min_bound, pP->x_max_bound);        //displacement
    NN.set_in_min_max(pP->x_dot_min_bound,pP->x_dot_max_bound); //velocity
    //NN.set_in_min_max(pP->x_dd_min_bound,pP->x_dd_max_bound); //velocity
    NN.set_out_min_max(pP->P_f_min_bound,pP->P_f_max_bound); // max forces
    NN.set_weights(pPo->P_weights, true);
    return NN;
}
double Simulator::set_P_force(neural_network NN, vector<double> state){
    NN.set_vector_input(state);
    NN.execute();
    pP->P_force = NN.get_output(0);
    assert(pP->P_force >= pP->P_f_min_bound - 0.5 && pP->P_force <= pP->P_f_max_bound + 0.5); //make sure matches NN output
    return pP->P_force;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
neural_network Simulator::set_A_NN(neural_network NNa, Policy* aPo){
    NNa.setup(pP->A_num_inputs,pP->num_nodes,pP->num_outputs);
    NNa.set_in_min_max(pP->x_min_bound, pP->x_max_bound);        //displacement
    NNa.set_in_min_max(pP->x_dot_min_bound,pP->x_dot_max_bound);  //velocity
    if (pP->A_num_inputs>=3){
        NNa.set_in_min_max(pP->P_f_min_bound, pP->P_f_max_bound); //Primary information
        if (pP->A_num_inputs==4){
            NNa.set_in_min_max(pP->A_f_min_bound, pP->A_f_max_bound); //Primary information
        }
    }
    
    NNa.set_out_min_max(pP->A_f_min_bound,pP->A_f_max_bound); // max forces
    NNa.set_weights(aPo->A_weights, true);
    return NNa;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void Simulator::zero_noise_sum_ave(){
    noise_x_sum = 0;
    noise_xdot_sum = 0;
    noise_sensor_sum = 0;
    noise_actuator_sum = 0;
    
    ave_actuator_noise = 0;
    ave_sensor_noise = 0;
    ave_x_noise = 0;
    ave_xdot_noise = 0;
    
    xt=0;
}
void Simulator::ave_noise(Policy* pPo, Policy* aPo){
    ave_x_noise = noise_x_sum/(2*pP->total_time);
    ave_xdot_noise = noise_xdot_sum/(2*pP->total_time);
    ave_sensor_noise = noise_sensor_sum/(2*pP->total_time);
    ave_actuator_noise = noise_actuator_sum/(2*pP->total_time);
    
    
    pPo->ave_position_noise_history.push_back(ave_x_noise);
    pPo->ave_velocity_noise_history.push_back(ave_xdot_noise);
    pPo->ave_sensor_noise_history.push_back(ave_sensor_noise);
    pPo->ave_actuator_noise_history.push_back(ave_actuator_noise);
}
void Simulator::sum_noise(vector<double> noise){
    noise_x_sum += noise.at(0) + noise.at(1);         //this is x noise total- sensor plus actuator
    noise_xdot_sum += noise.at(2) + noise.at(3);      //this is velocity noise total- sensor plus actuator
    noise_sensor_sum += noise.at(0) + noise.at(2);
    noise_actuator_sum += noise.at(1) + noise.at(3);
    
    noise_x_increm = noise.at(0) + noise.at(1);
    noise_xdot_increm = noise.at(2) + noise.at(3);
    noise_sensor_increm =noise.at(0) + noise.at(2);
    noise_actuator_increm = noise.at(1) + noise.at(3);
    
    //cout << "sum\t" << noise_x_sum << "\t" << noise_xdot_sum << "\t" << noise_sensor_sum << "\t" << noise_actuator_sum << endl;
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
        sinusoidal = pP->As*sin((pP->lambda)*(xt)+pP->phase);
        assert(sinusoidal <= abs(pP->As)+abs(r) && sinusoidal >= -abs(pP->As)-abs(r));
    }
    else{
        sinusoidal = 0;
    }
    double yt = sinusoidal + r;
    
    return yt;
}
double Simulator::xdot_sensor_noise(){
    double rr = generateGaussianNoise();
    assert(rr<=1 && rr>=-1);
    if (pP->sinusoidal_noise==true){
        sinusoidal = pP->As*sin((pP->lambda)*(xt)+pP->phase);
        assert(sinusoidal <= abs(pP->As)+abs(rr) && sinusoidal >= -abs(pP->As)-abs(rr));
    }
    else{
        sinusoidal = 0;
    }
    double yt = sinusoidal + rr;
    
    return yt;
}

double Simulator::x_actuator_noise(){
    double r = generateGaussianNoise();
    assert(r<=1 && r>=-1);
    
    if (pP->sinusoidal_noise==true){
        sinusoidal = pP->As*sin((pP->lambda)*(xt)+pP->phase);
        assert(sinusoidal <= abs(pP->As)+abs(r) && sinusoidal >= -abs(pP->As)-abs(r));
    }
    else{
        sinusoidal = 0;
    }
    double yt = sinusoidal + r;
    return yt;
}

double Simulator::xdot_actuator_noise(){
    double rr = generateGaussianNoise();
    assert(rr<=1 && rr>=-1);
    if (pP->sinusoidal_noise==true){
        sinusoidal = pP->As*sin((pP->lambda)*(xt)+pP->phase);
        assert(sinusoidal <= abs(pP->As)+abs(rr) && sinusoidal >= -abs(pP->As)-abs(rr));
    }
    else{
        sinusoidal = 0;
    }
    double yt = sinusoidal + rr;
    return yt;
}

vector<double> Simulator::set_sensor_actuator_noise(vector<double> noise){
    xt = xt+pP->dt;     //increment timer for the sinewave
    if (pP->sensor_NOISE == true) {
        noise.at(0)= x_sensor_noise();
        noise.at(2)= xdot_sensor_noise();
    }
    if (pP->actuator_NOISE == true) {
        noise.at(1)= x_actuator_noise();
        noise.at(3)= xdot_actuator_noise();
    }
    
    noise.at(0) = pP->sn*noise.at(0);           //sensor noise for position
    noise.at(1) = pP->an*noise.at(1);           //actuator noise for position
    noise.at(2) = pP->sn*noise.at(2);    //sensor noise for velocity
    noise.at(3) = pP->an*noise.at(3);           //actuator noise for velocity
    return noise;
}
vector<double> Simulator::noise_init(vector<double> noise){
    noise.push_back(0); // x sensor
    noise.push_back(0); // x actuator
    noise.push_back(0); // xdot sensor
    noise.push_back(0); // xdot actuator
    assert(noise.size()==4);
    return noise;
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void Simulator::MSD_equations(Policy* pPo, Policy* aPo){

    x_old = pPo->x;
    x_dot_old = pPo->x_dot;
    x_dd_old = pPo->x_dd;
    //pPo->x_dd = (1/(pP->m))*((-pP->b*pPo->x_dot) - (pP->k*(pPo->x-pP->start_x)) + pP->P_force + pP->A_force - pP->mu);
    pPo->x_dd = (1/(pP->m))*((-pP->b*x_dot_old) - (pP->k*(x_old-pP->start_x)) + pP->P_force + pP->A_force - pP->mu);
    //pPo->x_dot = pPo->x_dot + pPo->x_dd*pP->dt;
    pPo->x_dot = x_dot_old + pPo->x_dd*pP->dt;
    //pPo->x = pPo->x + pPo->x_dot*pP->dt;
    pPo->x = x_old + pPo->x_dot*pP->dt;
    if(pPo->x<0){
        pPo->x = 0;
        pPo->x_dot = 0;
    }
}
double Simulator::theta_dd(double theta){
    //pP->P_force=0;
    //pP->P_force= pP->P_force*pP->L*sin(theta);
    double c = pP->P_force + pP->A_force - pP->mu*x_dot_old - pP->L*9.81*pP->m*sin(theta);
    double theta_dd = 9.81*sin(theta) / (pP->L) + (c)/(pP->m*pow(pP->L,2)); //rad/s^2   // define theta_dd with t variable
    return theta_dd;
}
double Simulator::theta_dot(double theta_d){
    return theta_d;
}

void Simulator::Pendulum_equations(Policy *pPo, Policy *aPo){
    x_old = pPo->x;     // x is a representation of theta in this domain
    x_dot_old = pPo->x_dot;
    x_dd_old = pPo->x_dd;
    
    //double c = pP->P_force + pP->A_force - pP->mu*x_dot_old;
    //pPo->x_dd = 9.81*sin(x_old) / (pP->L) + (c)/(pP->m*pow(pP->L,2)); //rad/s^2   // define theta_dd with t variable
    //thetat_dd to theta_dot //EULER'S METHOD
    //pPo->x_dot = x_dot_old + pPo->x_dd*pP->dt;
    //theta_dot to theta
    //pPo->x = x_old + pPo->x_dot*pP->dt;
    
    //RK4
    t = t + pP->dt;
    double theta = x_old;
    double theta_d = x_dot_old;
    k1_w = theta_dd(theta);
    k1_theta = theta_dot(theta_d);
    pPo->x_dd = k1_w;
    
    theta = x_old + pP->dt*0.5*k1_theta;
    theta_d = x_dot_old + 0.5*pP->dt*k1_w;
    k2_w = theta_dd(theta);
    k2_theta = theta_dot(theta_d);
    
    theta = x_old + pP->dt*0.5*k2_theta;
    theta_d = x_dot_old + 0.5*pP->dt*k2_w;
    k3_w = theta_dd(theta);
    k3_theta = theta_dot(theta_d);
    
    theta = x_old + pP->dt*0.5*k3_theta;
    theta_d = x_dot_old + 0.5*pP->dt*k3_w;
    k4_w = theta_dd(theta);
    k4_theta = theta_dot(theta_d);
    
    pPo->x_dot = x_dot_old + (pP->dt/6)*(k1_w + 2*k2_w + 2*k3_w + k4_w);
    pPo->x = x_old + (pP->dt/6)*(k1_theta + 2*k2_theta + 2*k3_theta + k4_theta);
    
    /*
    if (pPo->x > 2*PI){
        pPo->x = pPo->x - 2*PI;
    }
    else if(pPo->x < -2*PI){
        pPo->x = 2*PI + pPo->x;
    }*/
    if (pPo->x > PI){
        pPo->x = pPo->x - 2*PI;
    }
    else if(pPo->x < -PI){
        pPo->x = 2*PI + pPo->x;
    }/*
    if (pPo->x > PI){
        pPo->x = PI;
        pPo->x_dot = 0;
    }
    else if(pPo->x < -PI){
        pPo->x = -PI;
        pPo->x_dot = 0;
    }*/
    assert(pPo->x <= PI && pPo->x >= -PI);
    
    //theta to xy
    //Px = L*cos(theta);
    //Py = L*sin(theta);
}
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void Simulator::calculateFitness(Policy* pPo, Policy* aPo, double remaining_ts){
    double ss_penalty = 0;
    
    if (pP->multi_var==true){
        pP->goal_x = pP->goal_x;//goal.at(k) from list
        //assert(pP->goal_x==aPo->A_ICs.at(0));
        //cout << pP->goal_x << endl;
        if (pP->Pend_EOM==true){
            
             if (pPo->x>PI/2 || pPo->x<-PI/2) {
                 ss_penalty = 10000*remaining_ts; //want closest to 0 displacement and penalize for not being at Steady state
             }
             
            double F_dist = (abs(pP->goal_x + abs(pPo->x))); //2 + resting position
            pPo->P_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
            aPo->A_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
        }
        else{
            double F_dist = (abs(pP->goal_x + pP->start_x - pPo->x)); //2 + resting position
            pPo->P_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
            aPo->A_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
        }
        
        
    }
    else {
        //pP->goal_x = 2;
        if (pP->Pend_EOM==true){
            if (pPo->x>PI/2 || pPo->x<-PI/2) {
                ss_penalty = 10000*remaining_ts; //want closest to 0 displacement and penalize for not being at Steady state
            }
            double F_dist = (abs(pP->goal_x + abs(pPo->x))); //2 + resting position
            assert(pP->goal_x == 0);
            pPo->P_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
            aPo->A_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
        }
        else{
            double F_dist = (abs(pP->goal_x + pP->start_x - pPo->x)); //2 + resting position
            pPo->P_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
            aPo->A_fit_swap += pP->w1*F_dist + pP->w2*ss_penalty;
            
        }
        
    }
    
}


//-------------------------------------------------------------------------
//Runs the entire simulation process
void Simulator::Simulate(Policy* pPo, Policy* aPo)
{
    pPo->P_fit_swap = 0;
    aPo->A_fit_swap = 0;
    t = t_init;
    assert(t==0);
    
    //NOISE GRAPH
    fstream nsensor, nactuator, tstep_sensor, tstep_actuator;
    nsensor.open("ave_sensor_noise.txt", fstream::app);
    nactuator.open("ave_actuator_noise.txt", fstream::app);
    tstep_sensor.open("tstep_sensor.txt", ofstream::out | ofstream::trunc);
    tstep_actuator.open("tstep_actuator.txt", ofstream::out | ofstream::trunc);
    
    //STARTING POSITIONS
    
    zero_noise_sum_ave();       //Zero outs all sums and averages of noise
    if (pP->MSD_EOM==true){
        MSD_initStates(pPo, aPo);   //Set Starting values
    }
    else if (pP->Pend_EOM==true){
        Pend_initStates(pPo,aPo);
    }
    // PROTAGONIST //
    neural_network NN;
    NN = set_P_NN(NN, pPo);     //Setup, set ins and outs, set weights
    
    // ANTAGONIST //
    neural_network NNa;
    if (pP->tr_3==true){
        NNa = set_A_NN(NNa, aPo);   //Setup, set ins and outs, set weights
    }

    //CLEAR x, xdot, xdd history vector
    clear_history(pPo, aPo);
    
    for (int i = 0; i < pP->total_time; i++) { // has to run long enough to change directions
        
        //give state vector to give to NN in order to update P_force
        vector<double> state;
        vector<double> noise;
        
        noise = noise_init(noise); //set sensor and actuator noise to zero
        assert(noise.at(0)==0 && noise.at(1)==0 && noise.at(2)==0 && noise.at(3)==0);
        
        noise = set_sensor_actuator_noise(noise); //if noise -> update noise
        
        state.push_back(pPo->x+noise.at(0)+noise.at(1));
        state.push_back(pPo->x_dot+noise.at(2)+noise.at(3));     //changed from x_dot to x_dd
        assert(state.size()==2);
        
        pP->P_force = set_P_force(NN, state);
        
    
        if (pP->rand_antagonist==true){
            pP->A_force = 0;
        }
        else if (pP->tr_3==true) {
            NNa.set_vector_input(state);
            NNa.execute();
            pP->A_force = NNa.get_output(0);
            assert(pP->A_force >= pP->A_f_min_bound - 0.5 && pP->A_force <= pP->A_f_max_bound + 0.5);
        }
        else{
            pP->A_force = 0;
        }
        // UPDATE POSITION, VELOCITY, ACCELERATION //
        if (pP->MSD_EOM==true){
            MSD_equations(pPo, aPo);
        }
        else if(pP->Pend_EOM==true){
            Pendulum_equations(pPo, aPo);
            
        }
       
        // CALCULATE FITNESS //
        double remaining_ts = pP->total_time-i;
        calculateFitness(pPo, aPo, remaining_ts);
        
        // SUM NOISE FOR POSITION, VELOCITY, SENSOR, AND ACTUATOR //
        sum_noise(noise);
        
        // STORE HISTORY (STATES,FORCES, NOISE) //
        history(pPo, aPo);
        if(pP->Pend_EOM==true){
            if (pPo->x>(PI/2) || pPo->x<(-PI/2)) {
                for (int j=0; j< pP->total_time-i-1; j++){
                    void_history(pPo, aPo);
                }
                i = pP->total_time-1;
            }
        }

    }
    
    ave_noise(pPo, aPo);
    

    tstep_sensor.close();
    tstep_actuator.close();
}


#endif /* Simulator_hpp */
