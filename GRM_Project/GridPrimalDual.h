#include <vector>
#include <array>
#include <stack>
#include <opencv2/opencv.hpp>
#include "Graph.h"
#include "GraphTree.h"

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
	int rows, columns, number_of_labels, distance_multiplier, dist_trunc, singleton_trunc;
	bool use_squared;
	cv::Mat image;
	std::vector < std::vector< int >> x;
	std::vector< std::vector< std::vector<FourNeighboors> > > y;
	std::vector < std::vector< std::vector< int >>> height;
	std::vector<std::vector<int>> pos2nodeIndex;
	GraphTree* g;


	std::vector<std::pair<int, int>> primal_dual_pair;

	//int optimize_step;
	
	GridPrimalDual(cv::Mat _image, int _number_of_labels, int _distance_multiplier, int _dist_trunc, int _singleton_trunc, bool _use_squared);
	void printPrimalDual();
	int distance(int a, int b);
	int singletonPotential(int row, int column, int c);
	void optimize();
	void preEditDuals(int c);
	void updateDualsPrimals(int c);
	void postEditDuals(int c);

	int compute_primal_cost();
	int compute_dual_cost();
	int compute_APF_cost();


	int compute_height_depth();

	void check();

private:
	int number_of_relabel;

	int truncatedSquaredDifference(int a, int b,int kappa, int truncation);
	int truncatedAbsoluteDifference(int a, int b, int kappa, int truncation);
	int label_height(int row, int column, int c);
	int label_height_diff(int row, int column, int a, int b);
	int load(int prow,int pcol,int qrow,int qcol,int a,int b);
};

