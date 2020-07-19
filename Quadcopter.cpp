// Quadcopter object 

#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "Quadcopter.h"

#define NUM_MOTORS (4)
#define NUM_STATES (12)
#define TIMESTEP (0.01f)

void Quadcopter::updateState()
{
	
	updateMotors();
	pos += vel * TIMESTEP;
	euler += eulerDot2rates.inverse() * omega * TIMESTEP;
	vel += 1 / mass * (gVec + R * motorForce) * TIMESTEP;
	omega += I.inverse() * (motorMoment - omega.cross(I * omega)) * TIMESTEP;
	updateR();
}

void Quadcopter::updateMotors()
{
	omegaM += km * (omegaMDes - omegaM) * TIMESTEP;
	motorForce << 0, 0, Kf * omegaM.array().pow(2).sum();
	motorMoment << L * (Kf * (pow(omegaM(1), 2) - pow(omegaM(3), 2))),
		L* (Kf * (pow(omegaM(2), 2) - pow(omegaM(0), 2))),
		Km* (pow(omegaM(0), 2) - pow(omegaM(1), 2) + pow(omegaM(2), 2) - pow(omegaM(3), 2));
}

// Update rotation matrices
void Quadcopter::updateR() 
{
	float phi{ euler(0) };
	float theta{ euler(1) };
	float psi{ euler(2) };
	R << cos(phi) * cos(theta) - sin(phi) * sin(psi) * sin(theta), -cos(phi) * sin(psi), cos(psi)* sin(theta) + cos(theta) * sin(phi) * sin(psi),
		cos(theta)* sin(psi) + cos(psi) * sin(phi) * sin(theta), cos(phi)* cos(psi), sin(psi)* sin(theta) - cos(psi) * cos(theta) * sin(phi),
		-cos(phi) * sin(theta), sin(phi), cos(phi) * cos(theta);

	eulerDot2rates << cos(theta), 0.f, -cos(phi) * sin(theta),
		0.f, 1.f, sin(phi),
		sin(theta), 0.f, cos(phi)* cos(theta);
}
