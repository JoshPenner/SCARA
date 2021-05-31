////////////////////////////////////////////////////////////////
// SCARA Robot Simulator
// Based on ELEX 7825 Template project for BCIT
// Created November 2, 2020 by Joshua Penner
// Last updated May 30, 2021
////////////////////////////////////////////////////////////////
#include "stdafx.h"

#include <string>
#include <iostream>
#include <thread>

// OpenCV 
#pragma comment(lib, ".\\opencv\\x64\\vc16\\lib\\opencv_world420d.lib") // debug
#include "opencv.hpp"

#include "CRobot.h"

using namespace cv;
using namespace std;

void print_menu();
void handle_menu_input(CRobot& robot);
void forward_kine_manual(CRobot& robot);
void forward_kine_anim(CRobot& robot);
void inverse_kine_manual(CRobot& robot);
void inverse_kine_anim(CRobot& robot);
void joint_traj_anim(CRobot& robot);
void cartesian_traj_anim(CRobot& robot);

//////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	CRobot Walle;

	while (1)
	{
		print_menu();
		handle_menu_input(Walle);
	}

  return 1;
}


void print_menu()
{
	std::cout << "Please select from menu options:" << std::endl;
	std::cout << "1) Forward kinematic manual mode" << std::endl;
	std::cout << "2) Forward kinematic animation" << std::endl;
	std::cout << "3) Inverse kinematic manual mode" << std::endl;
	std::cout << "4) Inverse kinematic animation" << std::endl;
	std::cout << "5) Joint trajectory animation" << std::endl;
	std::cout << "6) Cartesian trajectory animation" << std::endl;
	cout << endl;
}

void handle_menu_input(CRobot &robot)
{
	char menu_input;
	cin >> menu_input;
	switch (menu_input) 
	{
		case '1':
			forward_kine_manual(robot);
			break;
		case '2':
			forward_kine_anim(robot);
			break;
		case '3':
			inverse_kine_manual(robot);
			break;
		case '4':
			inverse_kine_anim(robot);
			break;
		case '5':
			joint_traj_anim(robot);
			break;
		case '6':
			cartesian_traj_anim(robot);
	}

}

void forward_kine_manual(CRobot& robot)
{
	float q1, q2, q3, q4;
	cout << endl;
	std::cout << "Enter angle for revolute joint 1: " << std::endl;
	std::cin >> q1;
	std::cout << "Enter angle for revolute joint 2: " << std::endl;
	std::cin >> q2;
	std::cout << "Enter angle for revolute joint 3: " << std::endl;
	std::cin >> q3;
	std::cout << "Enter distance for prismatic joint: " << std::endl;
	std::cin >> q4;
	cout << endl;

	robot.fkine(q1, q2, q3, q4);
}

void forward_kine_anim(CRobot& robot)
{
	cout << endl << "Animating" << endl << endl;
	robot.fkine_animate();
}

void inverse_kine_manual(CRobot& robot)
{
	double x, y, z, theta;
	
	cout << endl;
	std::cout << "Enter x coordinate: " << std::endl;
	std::cin >> x;
	std::cout << "Enter y coordinate: " << std::endl;
	std::cin >> y;
	std::cout << "Enter z coordinate: " << std::endl;
	std::cin >> z;
	std::cout << "Enter angle in degrees: " << std::endl;
	std::cin >> theta;
	cout << endl;

	robot.ikine(x, y, z, theta);
}

void inverse_kine_anim(CRobot& robot)
{
	cout << endl << "Animating" << endl << endl;

	double y = 50;
	double z = -50;
	double theta = 0;
	for (double x = -100; x <= 100; x = x + 5)
	{
		robot.ikine(x, y, z, theta);
	}
}

void joint_traj_anim(CRobot& robot)
{
	cout << endl << "Animating" << endl << endl;

	// jtraj variables
	float q1o, q1f, q2o, q2f, q3o, q3f, q4o, q4f;
	vector <float> q1v, q2v, q3v, q4v;
	int steps = 100;

	q1o = 0, q1f = 180;
	q2o = 0, q2f = 90;
	q3o = 0, q3f = 45;
	q4o = 0, q4f = 100;

	q1v = robot.jtraj(q1o, q1f, 0, 0, steps);
	q2v = robot.jtraj(q2o, q2f, 0, 0, steps);
	q3v = robot.jtraj(q3o, q3f, 0, 0, steps);
	q4v = robot.jtraj(q4o, q4f, 0, 0, steps);

	for (int i = 0; i <= steps; i++)
	{
		robot.fkine(q1v[i], q2v[i], q3v[i], q4v[i]);
	}
}

void cartesian_traj_anim(CRobot& robot)
{
	cout << endl << "Animating" << endl << endl;

	int steps = 100;

	Mat T1 = robot.createHT(400.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	Mat T2 = robot.createHT(-300.0, -200, -100.0, 0.0, 0.0, 45 * CV_PI / 180);
	Mat T3 = robot.createHT(200.0, 100.0, 0.0, 0.0, 0.0, 0.0);

	robot.ctraj(T1, T2, steps);
	robot.ctraj(T2, T3, steps);
}
