#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <limits>
#include <unordered_map>

#pragma once

class GraphTree
{
public:
	GraphTree(int _num_nodes, int _num_arcs_max);
	~GraphTree();
	void add_arc_pair(int p, int q, int cap, int reverse_cap);
	void add_terminal_cap(int node_p, bool source, int cap);
	void reset();

private:
	struct node;
	struct arc;


public:
	bool get_pflow(int node_p, int node_q, int& pflow);
	bool has_nonsat_path_to_source(int node_p);

	int max_flow();
	node* growth();
	int augment(node* node_p);
	void adopt();

private:
	int time_step;
	node* nodes;
	node* node_max;
	arc* arcs;
	arc* last_arc;
	std::queue<node*> active_nodes;
	std::queue<node*> orphan_nodes;
	int num_nodes, num_arcs, num_arcs_max;

	struct node
	{
		arc* first;		// first outcoming arc
		arc* parent;    // parent in the bfs tree. void if to source
		int time_step;  // time_step, kept as is for backward compatibility with the previous max flow implementation
		int distance_from_source;

		bool is_connected_to_source; // true if the cap/flow is for SOURCE->node, false if node->SINK
		int terminal_flow; //flow from source/to sink
		int	terminal_cap;  //cap from source/to sink
		bool in_source_tree;
	};

	struct arc
	{
		node* head;		// node the arc points to
		arc* next;		// next arc with the same originating node
		arc* sister;	// reverse arc (all arcs on the plane have a reverse, maybe be of 0 cap

		int pflow;		// pseudo flow (pflow == -sister->pflow)
		int	cap;		// capacity
	};


};

