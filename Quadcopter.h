// Function definitions and templates for Quadcopter.c

#pragma once

#include <Eigen/Dense>

#define NUM_MOTORS (4)
#define GRAVITY (9.8f)

using namespace Eigen;

enum State : int
{
	X = 0,
	Y,
	Z,
	PHI,
	THETA,
	PSI,
	VX,
	VY,
	VZ,
	P,
	Q,
	R
};

class Quadcopter
{
private:


public:
	Vector3f pos;
	Vector3f vel;
	Vector3f euler;
	Vector3f omega;
	Matrix3f R;
	Matrix3f eulerDot2rates;

	float mass{ 0.1f };
	Matrix3f I;
	Vector3f gVec;

	float Kf{ 6.11E-8f }; // N/rpm^2
	float Km{ 1.50E-9f }; // Nm/rpm^2
	float km{ 20.f }; // 1/s (motor time constant)
	float L{ 0.05f }; // Moment arm for motors
	VectorXf omegaM;
	VectorXf omegaMDes;
	Vector3f motorForce;
	Vector3f motorMoment;

	Quadcopter(float x0 = 0.f, float y0 = 0.f, float z0 = 0.f,
		float vx0 = 0.f, float vy0 = 0.f, float vz0 = 0.f)
	{
		// Initial state
		pos << x0, y0, z0;
		vel << vx0, vy0, vz0;
		euler << 0.0f, 0.f, 0.f;
		omega << 0.f, 0.f, 0.0f;
		updateR();
		
		// Mass properties
		mass = 0.1f; // kg
		I << 0.05f, 0.f, 0.f,
			0.f, 0.05f, 0.f,
			0.f, 0.f, 0.05f; // kg*m^2

		gVec << 0, 0, -mass* GRAVITY;

		// Motors
		omegaM = VectorXf::Zero(4);
		//omegaM    = VectorXf::Ones(4) * std::sqrt(mass*GRAVITY/(NUM_MOTORS*Kf)); // Hover rpm
		omegaMDes = VectorXf::Ones(4) * std::sqrt(mass * GRAVITY / (NUM_MOTORS * Kf)); // Hover rpm
		motorForce << 0, 0, Kf * omegaM.array().pow(2).sum();
		motorMoment << L*(Kf*(pow(omegaM(1), 2) - pow(omegaM(3), 2))),
			L* (Kf * (pow(omegaM(2), 2) - pow(omegaM(0), 2))),
			Km* (pow(omegaM(0), 2) - pow(omegaM(1), 2) + pow(omegaM(2), 2) - pow(omegaM(3), 2));


	}
	
	void updateState();
	void updateMotors();
	void updateR();
};
