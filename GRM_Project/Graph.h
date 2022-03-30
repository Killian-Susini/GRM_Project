#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <limits>
#include <unordered_map>


#pragma once
class Graph
{
public:
	Graph(int _num_nodes, int _num_arcs_max);
	~Graph();
	void add_arc_pair(int p, int q, int cap, int reverse_cap);
	void add_terminal_cap(int node_p, bool source, int cap);
	int max_flow();

private:
	struct node;
	struct arc;


public:
	node* find_augment();
	int augment(node* node_p);
	bool get_pflow(int node_p, int node_q, int& pflow);
	bool has_nonsat_path_to_source(int node_p);

private:
	int active_step;
	node* nodes;
	node* node_max;
	arc* arcs;
	arc* last_arc;

	int num_nodes, num_arcs, num_arcs_max;

	struct node
	{
		arc* first;		// first outcoming arc
		arc* parent;    // parent in the bfs tree. void if to source
		bool is_connected_to_source; // true if the cap/flow is for SOURCE->node, false if node->source
		int terminal_flow; //flow from source/to sink
		int	terminal_cap;  //cap from source/to sink


		int active; //used by find_augment, if equal to current active step then parent is used
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

