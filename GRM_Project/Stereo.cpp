#include "Stereo.h"

Stereo::Stereo(int _half_win_size, int _max_disparity)
{
	half_win_size = _half_win_size;
	max_disparity = _max_disparity;
}

std::vector<uint8_t> Stereo::ssd(Mat& left, Mat& right)
{
	int cumul, diff;
	int rows = left.rows;
	int cols = left.cols;
	std::vector<std::vector<int>> best_disparity_score;
	std::vector<uint8_t>  best_disparity = std::vector<uint8_t>(rows * cols, 0);
	best_disparity_score.reserve(rows);
	for (int i = 0; i < cols; i++) {
		best_disparity_score.push_back(std::vector<int>(cols, INT_MAX));
	}

	for (uint8_t d = 0; d < max_disparity; d++) {
		for (int row = half_win_size; row < rows - half_win_size; row++) {
			for (int col = half_win_size; col < cols - half_win_size - d; col++){
				cumul = 0;
				for (int y = -half_win_size; y <= half_win_size; y++) {
					for (int x = -half_win_size; x <= half_win_size; x++) {

						diff = left.at<uchar>(row + x, col + y) - right.at<uchar>(row + x, col + y + d);
						cumul += diff * diff;
					}
				}
				if (best_disparity_score[row][col] > cumul) {
					best_disparity_score[row][col] = cumul;
					best_disparity[row*cols + col] = d;
				}
			}
		}
	}
	return best_disparity;
}
