#include <vector>
#include <array>
#include <stack>
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
	int rows, columns, number_of_labels, distance_multiplier;
	cv::Mat image;
	std::vector < std::vector< int >> x;
	std::vector< std::vector< std::vector<FourNeighboors> > > y;
	
	GridPrimalDual(cv::Mat _image, int _number_of_labels, int _distance_multiplier);
	void printPrimalDual();
	int distance(int a, int b);
	int singletonPotential(int row, int column, int c);
	void optimize();
	void preEditDuals(int c);
	void updateDualsPrimals(int c);
	void postEditDuals(int c);

private:
	int number_of_relabel;

	int truncatedSquaredDifference(int a, int b,int kappa, int truncation);
	int label_height(int row, int column, int c);
	int load(int prow,int pcol,int qrow,int qcol,int a,int b);

	void recomputeHeights(std::vector<std::vector<int>>& height, std::vector<std::vector<NodesNeighboors>>& C, std::vector<std::vector<NodesNeighboors>>& F);

	void push2Sink(int prow, int pcol,
		std::vector<std::vector<int>>& excess, 
		std::vector<std::vector<NodesNeighboors>>& C, 
		std::vector<std::vector<NodesNeighboors>>& F);
	void pushFromSource(int qrow, int qcol, 
		std::vector<std::vector<int>>& excess, 
		std::vector<std::vector<int>>& source2p_capacity, 
		std::vector<std::vector<int>>& source2p_flow, 
		std::vector<std::vector<NodesNeighboors>>& F);
	void push2Source(int prow, int pcol, 
		std::vector<std::vector<int>>& excess, 
		std::vector<std::vector<int>>& source2p_flow,
		std::vector<std::vector<NodesNeighboors>>& C,
		std::vector<std::vector<NodesNeighboors>>& F);
	
	void relabel(int prow, int pcol, 
		std::vector<std::vector<int>>& height, 
		std::vector<std::vector<NodesNeighboors>>& C, 
		std::vector<std::vector<NodesNeighboors>>& F);
	void discharge(int prow, int pcol, 
		std::vector<std::vector<int>>& excess, 
		std::vector<std::vector<int>>& height,
		std::vector<std::vector<int>>& source2p_capacity, 
		std::vector<std::vector<int>>& source2p_flow, 
		std::vector<std::vector<NodesNeighboors>>& C, 
		std::vector<std::vector<NodesNeighboors>>& F);
};

