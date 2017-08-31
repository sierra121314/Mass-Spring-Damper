//
//  test.cpp
//  Project
//
//  Created by Sierra Gonzales on 8/31/17.
//  Copyright © 2017 Sierra Gonzales. All rights reserved.
//

#include <stdio.h>
#include <iostream>

using namespace std;

int main () {
    
    double m = 1;
    double b = 1;
    double k = 1;
    
    double x_dd;
    double x_dot = 1;
    double x =1;
    double force =1;
    double mu = 0;
    double dt = 0.01;
    
    x_dd = (1/m)*(-b*x_dot + k*x +force - mu);
    // x_dd to x_dot
    x_dot = x_dot + x_dd*dt;
    // x_dot to x
    x = x + x_dot*dt;
    
    cout << x_dd << "," << x_dot << "," << x << endl;

    
}


              