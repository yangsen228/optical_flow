#include <iostream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "include/utils/get_fps.h"

using namespace std;
using namespace cv;

#define kernel_size 3
#define kernel_step 35
int g_fps;
GetFps get_fps;

// calculate the sum of converted vector
void calcSumOfArray(const Eigen::ArrayXf &I, float &sum)
{
	cout << "calcSumOfArray start ==================" << endl;

	cout << "I*I = " << I << endl;

	sum = 0;
	for(int i = 0; i < I.size(); i++){
		sum += I(i);
	}

	cout << "calcSumOfArray end ==================" << endl;	
}

// convert the sampling Mat matrix to a vector
void convertMatToVector(int col, int row, const Mat &gradient, Eigen::ArrayXf &I)
{
	cout << "convertMatToVector start =============" << endl;

	int k = 0;
	int start_col = col - (kernel_size - 1)/2;
	int start_row = row - (kernel_size - 1)/2;

	cout << "start_col = " << start_col << " start_row = " << start_row << endl;

	for(int i = start_col; i < start_col + kernel_size; i++){
		for(int j = start_row; j < start_row + kernel_size; j++){
			//cout << "i = " << i << " j = " << j << endl;
			I(k) = gradient.at<uchar>(j, i);
			k++;
			//cout << "gradient_x = " << I(k) << endl;
		}
	}
	cout << "I = " << I << endl;

	cout << "convertMatToVector end =============" << endl;
}

// calculate the velocity of each sampling point
void calcVelocity(int col, int row, const Mat &gradient_x, const Mat &gradient_y, const Mat gradient_t, vector<float> &velocity)
{
	cout << "calcVelocity start --------" << endl;

	cout << "col = " << col << ", row = " << row << endl;

	// convert the gradients of sampling Mat matrix to vectors 
	int num = pow(kernel_size, 2);
	Eigen::ArrayXf Ix(num), Iy(num), It(num);
	convertMatToVector(col, row, gradient_x, Ix);
	convertMatToVector(col, row, gradient_y, Iy);
	convertMatToVector(col, row, gradient_t, It);

	// calculate the elements of matrix A and vector z
	float A11, A12, A22, z1, z2;
	calcSumOfArray(Ix*Ix, A11);
	calcSumOfArray(Ix*Iy, A12);
	calcSumOfArray(Iy*Iy, A22);
	calcSumOfArray(-Ix*It, z1);
	calcSumOfArray(-Iy*It, z2);

	// claculate the velocity (v = A^-1 * z)
	Eigen::Matrix2f A;
	Eigen::Vector2f current_velocity, z;
	A << A11, A12,
		 A12, A22;
	z << z1, z2;
	cout << "A = " << A << endl << "z = " << z << endl;
	current_velocity = A.transpose() * z;

	velocity.push_back(current_velocity(0));
	velocity.push_back(current_velocity(1));
	cout << "velocity" << current_velocity(0) << ", " << current_velocity(1) << endl;

	cout << "calcVelocity end ---------" << endl;
}

// visualize optical flow image
void calcOpticalFlow(uint width, uint height, const Mat &gradient_x, const Mat &gradient_y, const Mat &gradient_t, Mat &result)
{
	cout << "calcOpticalFlow start ......" << endl;

	uint first_x = (kernel_size + 1) / 2;
	uint first_y = (kernel_size + 1) / 2;
	cout << "first = " << first_x << " " << first_y << endl;

	vector<float> velocity;
	for(int i = first_x; i < width; i += kernel_step){
		cout << "--------------------------" << endl;
		for(int j = first_y; j < height; j += kernel_step){
			calcVelocity(i, j, gradient_x, gradient_y, gradient_t, velocity);
		}
	}	
	normalize(velocity, velocity);
	int k = 0;
	for(int i = first_x; i < width; i += kernel_step){
		for(int j = first_y; j < height; j += kernel_step){
			cout << "velocity = " << velocity[0] << " " << velocity[1] << endl;
			line(result, Point(i, j), Point(i+20*velocity[2*k], j+20*velocity[2*k+1]), Scalar(0,0,255), 2, 8);
			k++;
		}
	}	
	velocity.clear();

	cout << "calcOpticalFlow end ......" << endl;
}

int main()
{
	VideoCapture capture(1);
	// kernels of x and y gradients
	Mat kernel_x = (Mat_<int>(3, 3) << -1, 0, 1,
									   -2, 0, 2,
									   -1, 0, 1);
	Mat kernel_y = (Mat_<int>(3, 3) << -1, -2, -1,
									   0, 0, 0,
									   1, 2, 1);
	Mat pre_frame, A;
	capture >> A;
	int width = A.size().width;
	int height = A.size().height;

	while (1)
	{
		Mat frame;
		capture >> frame;
		
		// convert to grayscale image
		Mat gray;
		cvtColor(frame, gray, COLOR_BGR2GRAY);
		imshow("origin", gray);

		// calculate the gradient in the direction of x
		Mat gradient_x;
		filter2D(gray, gradient_x, -1, kernel_x);
		imshow("gradient_x", gradient_x);

		// calculate the gradient in the direction of y
		Mat gradient_y;
		filter2D(gray, gradient_y, -1, kernel_y);
		imshow("gradient_y", gradient_y);

		// first frame
		if (pre_frame.empty()) {
			pre_frame = gray.clone();
		}
		else {
			// calculate the time gradient
			Mat gradient_t;
			absdiff(gray, pre_frame, gradient_t);
			threshold(gradient_t, gradient_t, 25, 255, CV_THRESH_TOZERO);
			Mat dilate_element = getStructuringElement(MORPH_RECT, Size(20,20));	
			Mat erode_element = getStructuringElement(MORPH_RECT, Size(3,3));	
			erode(gradient_t, gradient_t, erode_element);
			dilate(gradient_t, gradient_t, dilate_element);
			imshow("gradient_t", gradient_t);

			// obtain the result image
			Mat result(frame);
			calcOpticalFlow(width, height, gradient_x, gradient_y, gradient_t * g_fps, result);

			// draw fps
			get_fps.getFps(g_fps);
			string s_fps = to_string(g_fps);
			putText(result, "fps = "+s_fps, Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 1, Scalar(100,100,255), 2, 8);
			imshow("result", result);

			pre_frame = gray.clone();
		}

		if (waitKey(30) == 27) {
			break;
		}
	}

	return 0;
}
