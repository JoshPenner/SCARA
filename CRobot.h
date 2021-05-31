#include "stdafx.h"
#include "opencv.hpp"

using namespace cv;
using namespace std;

#pragma once
class CRobot
{
private:
	Size image_size = Size(1000, 600);
	cv::Mat _im;

	// Tilt the view for better 3D look
	float view_angle = -1.8;
	Mat W, T1, T0_1, T2, T1_2, T3, T2_3, T4, T3_4, E;

	std::vector <Mat> _link1;
	std::vector <Mat> _link2;
	std::vector <Mat> _link3;

public:
	CRobot();
	~CRobot();

	void fkine(float q1, float q2, float q3, float q4);
	void fkine_animate();
	void ikine(double x, double  y, double z, double  theta);
	vector<float> jtraj(double so, double sf, double vo, double vf, double T);
	void ctraj(Mat To, Mat Tf, float steps);

	virtual void plot();

	cv::Mat createHT(float tx, float ty, float tz, float rx, float ry, float rz);
	std::vector<Mat> createBox(float w, float h, float d);
	void transformBox(std::vector<Mat>& box, Mat T);
	void drawBox(std::vector<Mat> box, Scalar colour, Mat T);
	void drawPose(Mat T);
	
	Vec3f rotationMatrixToEulerAngles(Mat& R);
};

