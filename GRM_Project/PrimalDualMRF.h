#include <functional>
#include <string>
#include <opencv2/opencv.hpp>
#include "Graph.h"

using namespace std;

#pragma once
class PrimalDualMRF
{
private:
	struct node;
	struct arc;
public:
	int rows, cols, number_of_labels, num_nodes, num_arcs, num_arcs_max;
	vector<int> orig_values;
	cv::Mat image;
	node*  x, * x_max;
	arc*   y, * last_arc; 
	Graph* g;
	function<int(int,int)> distanceFunction, singletonDistanceFunction;
	
	PrimalDualMRF(cv::Mat _image, int _number_of_labels, function<int(int, int)> _distanceFunction, function<int(int, int)> _singletonDistanceFunction);
	~PrimalDualMRF();
	void add_arcs_pair(int node_p, int node_q);
	void optimize();
	void preEditDuals(int c);
	void updateDualsPrimals(int c);
	void postEditDuals(int c);
	int label_height(node* p, int c);
	int load(arc* pq, int a, int b);



private:
	struct node
	{
		arc* first; 
		int x;
		int x_old_label;
		int id;
	};

	struct arc
	{
		node* head;
		arc* sister;	// reverse arc (all arcs on the plane have a reverse) (for easy update)
		arc* next; //next for same c
		int* y; //per c
	};
};

