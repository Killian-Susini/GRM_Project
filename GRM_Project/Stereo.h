#include <opencv2/opencv.hpp>
using namespace cv;
#pragma once
class Stereo
{
public:
	Stereo(int _half_win_size, int _max_disparities);
	std::vector<uint8_t>  ssd(Mat &left, Mat &right);

	int half_win_size, max_disparity;
};

