#include <vector>
#include <array>
#include <opencv2/opencv.hpp>

struct FourNeighboors
{
	int right, down; //, left, up;  ypq(a) == -yqp(a) always (conjugate). so left is negative right of node left
};

struct NodesNeighboors {
	int right, down, left, up, source, sink;
};

#pragma once
class GridPrimalDual
{
public:
	int rows, columns, number_of_labels;
	cv::Mat image;
	std::vector < std::vector< int >> x;
	std::vector< std::vector< std::vector<FourNeighboors> > > y;
	
	GridPrimalDual(cv::Mat _image, int _number_of_labels);
	void printPrimalDual();
	int distance(int a, int b);
	int singletonPotential(int row, int column, int c);

	void preEditDuals(int c);
	void updateDualsPrimal(int c);
	void postEditDuals(int c);

private:
	int truncatedSquaredDifference(int a, int b, int truncation);
	int label_height(int row, int column, int c);
	int load(int prow,int pcol,int qrow,int qcol,int a,int b);
};

