#include "stdafx.h"
#include "CRobot.h"

#include<cmath>

#include <chrono>
#include <thread>

#include <iomanip>

using namespace cv;
using namespace std;

CRobot::CRobot()
{
}

CRobot::~CRobot()
{
}

// Create Homogeneous Transformation Matrix
Mat CRobot::createHT(float tx, float ty, float tz, float rx, float ry, float rz)
{
	float r11, r12, r13, r21, r22, r23, r31, r32, r33;

	r11 = cos(rz) * cos(ry);
	r12 = cos(rz) * sin(ry) * sin(rx) - sin(rz) * cos(rx);
	r13 = cos(rz) * sin(ry) * cos(rx) + sin(rz) * sin(rx);
	r21 = sin(rz) * cos(ry);
	r22 = sin(rz) * sin(ry) * sin(rx) + cos(rz) * cos(rx);
	r23 = sin(rz) * sin(ry) * cos(rx) - cos(rz) * sin(rx);
	r31 = -1.0 * sin(ry);
	r32 = cos(ry) * sin(rx);
	r33 = cos(ry) * cos(rx);

	return (Mat1f(4, 4) << r11, r12, r13, tx, r21, r22, r23, ty, r31, r32, r33, tz, 0, 0, 0, 1);
}

std::vector<Mat> CRobot::createBox(float w, float h, float d)
{
	std::vector <Mat> box;

	// The 8 vertexes, origin at the middle of the box
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, -d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, -h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << w / 2, h / 2, d / 2, 1)));
	box.push_back(Mat((Mat1f(4, 1) << -w / 2, h / 2, d / 2, 1)));

	// Move origin to middle of the the left hand face
	Mat T = createHT(w / 2, 0, 0, CV_PI, 0, 0);
	for (int i = 0; i < box.size(); i++)
	{
		box.at(i) = T * box.at(i);
	}

	return box;
}

void CRobot::transformBox(std::vector<Mat>& box, Mat T)
{
	for (int i = 0; i < box.size(); i++)
	{
		box.at(i) = T * box.at(i);
	}
}

void CRobot::drawBox(std::vector<Mat> box, Scalar colour, Mat T)
{
	// The 12 lines connecting all vertexes 
	float draw_box1[] = { 0,1,2,3,4,5,6,7,0,1,2,3 };
	float draw_box2[] = { 1,2,3,0,5,6,7,4,4,5,6,7 };

	transformBox(box, T);

	//drawPose(im, T);

	for (int i = 0; i < 12; i++)
	{
		Point pt1 = Point2f(box.at(draw_box1[i]).at<float>(0, 0), box.at(draw_box1[i]).at<float>(1, 0));
		Point pt2 = Point2f(box.at(draw_box2[i]).at<float>(0, 0), box.at(draw_box2[i]).at<float>(1, 0));
		line(_im, pt1, pt2, colour, 1);
	}
}

void CRobot::drawPose(Mat T)
{
	std::vector <Mat> pose;

	// The 8 vertexes, origin at the middle of the box
	pose.push_back(Mat((Mat1f(4, 1) << 0, 0, 0, 1)));
	pose.push_back(Mat((Mat1f(4, 1) << 10, 0, 0, 1)));
	pose.push_back(Mat((Mat1f(4, 1) << 0, 10, 0, 1)));
	pose.push_back(Mat((Mat1f(4, 1) << 0, 0, 10, 1)));

	for (int i = 0; i < pose.size(); i++)
	{
		pose.at(i) = T * pose.at(i);
	}

	float draw_pose1[] = { 0,0,0};
	float draw_pose2[] = { 1,2,3};
	
	Point pt1;
	Point pt2;

	// x-axis
	pt1 = Point2f(pose.at(draw_pose1[0]).at<float>(0, 0), pose.at(draw_pose1[0]).at<float>(1, 0));
	pt2 = Point2f(pose.at(draw_pose2[0]).at<float>(0, 0), pose.at(draw_pose2[0]).at<float>(1, 0));
	line(_im, pt1, pt2, CV_RGB(255, 0, 0), 1);

	// y-axis
	pt1 = Point2f(pose.at(draw_pose1[1]).at<float>(0, 0), pose.at(draw_pose1[1]).at<float>(1, 0));
	pt2 = Point2f(pose.at(draw_pose2[1]).at<float>(0, 0), pose.at(draw_pose2[1]).at<float>(1, 0));
	line(_im, pt1, pt2, CV_RGB(0, 255, 0), 1);

	// z-axis
	pt1 = Point2f(pose.at(draw_pose1[2]).at<float>(0, 0), pose.at(draw_pose1[2]).at<float>(1, 0));
	pt2 = Point2f(pose.at(draw_pose2[2]).at<float>(0, 0), pose.at(draw_pose2[2]).at<float>(1, 0));
	line(_im, pt1, pt2, CV_RGB(0, 0, 255), 1);
}

void CRobot::fkine(float q1, float q2, float q3, float q4)
{
	// World coordinates, relocated to center and view angle adjusted
	W = createHT(_im.size().width / 2, _im.size().height / 2, 0, CV_PI + view_angle, 0, 0);

	// revolute joint 1
	T0_1 = createHT(0, 0, 0, 0, 0, q1 * CV_PI / 180);
	T1 = W * T0_1;
	// link 1
	_link1 = createBox(200, 50, 50);

	// revolute joint 2
	T1_2 = createHT(200, 0, 0, 0, 0, q2 * CV_PI / 180);
	T2 = T1 * T1_2;
	// link 2
	_link2 = createBox(200, 50, 50);

	// revolute joint 3
	T2_3 = createHT(200, 0, 0, 0, -CV_PI / 2, q3 * CV_PI / 180);
	T3 = T2 * T2_3;

	// prismatic joint
	T3_4 = createHT(-q4, 0, 0, 0, 0, 0);
	T4 = T3 * T3_4;
	// link 3
	_link3 = createBox(200, 50, 50);

	//cout << std::fixed << std::setprecision(3) << T0_1 * T1_2 * T2_3 * T3_4 << "\n\n";
	E = T0_1 * T1_2 * T2_3 * T3_4;

	std::cout.setf(std::ios::fixed, std::ios::floatfield);
	std::cout.precision(3);
	for (int y = 0; y < E.rows; ++y) {
		for (int x = 0; x < E.cols; ++x) {
			std::cout << E.at<float>(y, x) << "\t";
		}
		cout << endl;
	}
	cout << endl;

	plot();

}

void CRobot::ikine(double x, double y, double z, double theta)
{
	double a1 = 200;
	double a2 = 200;

	float q1,q2,q3,q4;

	//q1 in radians
	q1 = 2.0 * atan((2.0 * a1 * y + pow((-pow(a1, 4.0) + 2.0 * pow(a1, 2.0) * pow(a2, 2.0) + 2.0 * pow(a1, 2.0) * pow(x, 2.0) + 2.0 * pow(a1, 2.0) * pow(y, 2.0) - pow(a2, 4.0) + 2.0 * pow(a2, 2.0) * pow(x, 2.0) + 2.0 * pow(a2, 2.0) * pow(y, 2.0) - pow(x, 4.0) - 2.0 * pow(x, 2.0) * pow(y, 2.0) - pow(y, 4.0)) , 0.5 )) / (pow(a1, 2.0) + 2.0 * a1 * x - pow(a2, 2.0) + pow(x, 2.0) + pow(y, 2.0)));
	//q1 in degrees
	q1 = q1 * 180.0 / CV_PI;

	//q2 in radians
	q2 = -2.0 * atan(pow((-pow(a1, 2.0) + 2.0 * a1 * a2 - pow(a2, 2.0) + pow(x, 2.0) + pow(y, 2.0)) * (pow(a1, 2.0) + 2 * a1 * a2 + pow(a2, 2.0) - pow(x, 2.0) - pow(y, 2.0)), 0.5) / (-pow(a1, 2.0) + 2.0 * a1 * a2 - pow(a2, 2.0) + pow(x, 2.0) + pow(y, 2.0)));
	//q2 in degrees
	q2 = q2 * 180.0 / CV_PI;

	//q3 in degrees
	q3 = theta - (q1 + q2);

	// q4 distance
	q4 = -z;

	//cout << "q1 " << q1 << endl;
	//cout << "q2 " << q2 << endl;
	//cout << "q3 " << q3 << endl;
	//cout << "q4 " << q4 << endl << endl;

	fkine(q1, q2, q3, q4);
}

vector<float> CRobot::jtraj(double so, double sf, double vo, double vf, double T)
{
	double ao = 0;
	double af = 0;

	Mat polyMat, sVector, sCoeffs;

	std::vector <float> s;

	polyMat = (Mat1f(6, 6) << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, pow(T, 5.0), pow(T, 4.0), pow(T, 3.0), pow(T, 2.0), T, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, 5.0 * pow(T, 4.0), 4.0 * pow(T, 3.0), 3.0 * pow(T, 2.0), 2.0 * T, 1.0, 0.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 20.0 * pow(T, 3), 12.0 * pow(T, 2), 6.0 * T, 2.0, 0.0, 0.0);

	sVector = (Mat1f(6, 1) << so, sf, vo, vf, ao, af);

	sCoeffs = polyMat.inv() * sVector;

	float A = sCoeffs.at<float>(0, 0);
	float B = sCoeffs.at<float>(1, 0);
	float C = sCoeffs.at<float>(2, 0);
	float D = sCoeffs.at<float>(3, 0);
	float E = sCoeffs.at<float>(4, 0);
	float F = sCoeffs.at<float>(5, 0);

	for (float t = 0; t <= T; t++)
	{
		s.push_back(A * pow(t, 5.0) + B * pow(t, 4.0) + C * pow(t, 3.0) + D * pow(t, 2.0) + E * t + F);
	}
	return s;
}

Vec3f CRobot::rotationMatrixToEulerAngles(Mat& R)
{
	float sy = sqrt(R.at<float>(0, 0) * R.at<float>(0, 0) + R.at<float>(1, 0) * R.at<float>(1, 0));
	bool singular = sy < 1e-6; // If
	float roll, pitch, yaw;
	if (!singular)
	{
		roll = atan2(R.at<float>(2, 1), R.at<float>(2, 2));
		pitch = atan2(-R.at<float>(2, 0), sy);
		yaw = atan2(R.at<float>(1, 0), R.at<float>(0, 0));
	}
	else
	{
		roll = atan2(-R.at<float>(1, 2), R.at<float>(1, 1));
		pitch = atan2(-R.at<float>(2, 0), sy);
		yaw = 0;
	}
	return Vec3f(roll, pitch, yaw);
}

void CRobot::ctraj(Mat To, Mat Tf, float steps)
{
	Vec3f Ro = rotationMatrixToEulerAngles(To);
	Vec3f Rf = rotationMatrixToEulerAngles(Tf);

	vector <float> xv, yv, zv, thetav;

	float xo = To.at<float>(0, 3);
	float yo = To.at<float>(1, 3);
	float zo = To.at<float>(2, 3);
	float thetao = Ro[2];

	float xf = Tf.at<float>(0, 3);
	float yf = Tf.at<float>(1, 3);
	float zf = Tf.at<float>(2, 3);
	float thetaf = Rf[2];

	xv = jtraj(xo, xf, 0, 0, steps);
	yv = jtraj(yo, yf, 0, 0, steps);
	zv = jtraj(zo, zf, 0, 0, steps);
	thetav = jtraj(thetao, thetaf, 0, 0, steps);

	for (int i = 0; i <= steps; i++)
	{
		ikine(xv[i], yv[i], zv[i], thetav[i]);
	}
}


void CRobot::plot()
{

	cv::namedWindow("7825 Project");
	_im = cv::Mat::zeros(image_size, CV_8UC3) + CV_RGB(60, 60, 60);

	// revolute joint 1
	drawPose(T1);
	// link 1 - red
	drawBox(_link1, CV_RGB(255, 0, 0), T1);

	// revolute joint 2
	drawPose(T2);
	// link 2 - green
	drawBox(_link2, CV_RGB(0, 255, 0), T2);

	// revolute joint 3
	drawPose(T3);

	// prismatic joint
	drawPose(T4);
	// link 3 - blue
	drawBox(_link3, CV_RGB(0, 0, 255), T4);

	imshow("7825 Project", _im);

	cv::waitKey(70);
}

void CRobot::animate()
{

	float q1=0, q2=0, q3=0, q4=0;

	for (q1 = 0; q1 < 360; q1 = q1 + 10)
	{
		fkine(q1, q2, q3, q4);
	}

	for (q2 = 0; q2 < 360; q2 = q2 + 10)
	{
		fkine(q1, q2, q3, q4);
	}

	for (q3 = 0; q3 < 360; q3 = q3 + 10)
	{
		fkine(q1, q2, q3, q4);
	}

	for (q4 = 0; q4 <= 100; q4 = q4 + 10)
	{
		fkine(q1, q2, q3, q4);
	}

	//cv::waitKey(0);
}