////////////////////////////////////////////////////////////////
// ELEX 7825 Template project for BCIT
// Created Sept 9, 2020 by Craig Hennessey
// Last updated Sept 9, 2020
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

//////////////////////////////////////////////////////////////////////////////////////////////////
//

int main(int argc, char* argv[])
{
	char menu_input;

	while (1)
	{
		float q1, q2, q3, q4;
		double x, y, z, theta;
		Mat T1, T2, T3;

		// jtraj variables
		float q1o, q1f, q2o, q2f, q3o, q3f, q4o, q4f;
		vector <float> q1v, q2v, q3v, q4v;
		float steps;


		CRobot Walle;
		
		print_menu();

		cin >> menu_input;
		switch (menu_input) {
		case '1':
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

			Walle.fkine(q1, q2, q3, q4);

			break;

		case '2':
			cout << endl;
			std::cout << "Animating" << std::endl;
			cout << endl;
			Walle.animate();
			break;
		case '3':
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

			Walle.ikine(x, y, z, theta);

			break;

		case '4':
			cout << endl;
			std::cout << "Animating" << std::endl;
			cout << endl;
			y = 50;
			z = -50;
			theta = 0;
			for (x = -100; x <= 100; x=x+5)
			{
				Walle.ikine(x, y, z, theta);
			}

			break;

		case '5':

			q1o = 0,	q1f = 180;
			q2o = 0, q2f = 90;
			q3o = 0, q3f = 45;
			q4o = 0, q4f = 100;

			steps = 100;

			q1v = Walle.jtraj(q1o, q1f, 0, 0, steps);
			q2v = Walle.jtraj(q2o, q2f, 0, 0, steps);
			q3v = Walle.jtraj(q3o, q3f, 0, 0, steps);
			q4v = Walle.jtraj(q4o, q4f, 0, 0, steps);

			for (int i = 0; i <= steps; i++)
			{
				Walle.fkine(q1v[i], q2v[i], q3v[i], q4v[i]);
			}

			break;

		case '6':

			T1 = Walle.createHT(400.0,		0.0,	0.0,		0.0,	0.0,	0.0);
			T2 = Walle.createHT(-300.0,		-200,	-100.0,		0.0,	0.0,	45 * CV_PI / 180);
			T3 = Walle.createHT(200.0,		100.0,	0.0,		0.0,	0.0,	0.0);

			steps = 100;
			Walle.ctraj(T1, T2, steps);
			Walle.ctraj(T2, T3, steps);

		}
	}


  //draw_opencv_box();


	
  return 1;
}


void print_menu()
{
	std::cout << "Please select from menu options:" << std::endl;
	std::cout << "1) Forward kine manual mode" << std::endl;
	std::cout << "2) Animate - fkine" << std::endl;
	std::cout << "3) Inverse kine" << std::endl;
	std::cout << "4) Animate - ikine" << std::endl;
	std::cout << "5) Jtraj" << std::endl;
	std::cout << "6) Ctraj" << std::endl;
	cout << endl;
}
