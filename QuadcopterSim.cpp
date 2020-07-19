// QuadcopterSim.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <Eigen/Dense>
#include "Quadcopter.h"


int main()
{
    Quadcopter quad(1.f, 1.f);
    
    std::cout << quad.euler(0) << std::endl;
    quad.updateState();
    std::cout << quad.euler(0) << std::endl;
    quad.updateState();
    std::cout << quad.euler(0) << std::endl;
    quad.updateState();
    std::cout << quad.euler(0) << std::endl;
    quad.updateState();
    std::cout << quad.euler(0) << std::endl;
    quad.updateState();
    std::cout << quad.euler(0) << std::endl;
    quad.updateState();
    std::cout << quad.euler(0) << std::endl;
    quad.updateState();




    
}