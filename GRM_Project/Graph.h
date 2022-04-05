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
	void reset();

private:
	struct node;
	struct arc;


public:
	node* find_augment();
	int augment(node* node_p);
	bool get_pflow(int node_p, int node_q, int& pflow);
	bool has_run();
	bool has_nonsat_path_to_source(int node_p);

	int max_flow_push_relabel();
	void push(node* u, arc* uv);
	void relabel(node* u);
	void discharge(node* u);

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
		bool is_connected_to_source; // true if the cap/flow is for SOURCE->node, false if node->SINK
		int terminal_flow; //flow from source/to sink
		int	terminal_cap;  //cap from source/to sink


		int active; // if augmenting path style, it is used to check if already seen by the search. if push relabel, it keeps the excess
		int label; //used by push relabel max flow
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

