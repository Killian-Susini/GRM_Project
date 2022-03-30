#include <iostream>
#include <vector>
#include <list>
#include <queue>
#include <stack>
#include <limits>
#include <unordered_map>

#include "Graph.h"



Graph::Graph(int _num_nodes, int _num_arcs_max)
{
	num_nodes = _num_nodes;
	num_arcs_max = _num_arcs_max;
	nodes = (node*)malloc(num_nodes * sizeof(node));
	arcs = (arc*)malloc(num_arcs_max * sizeof(arc));
	if (!nodes || !arcs) { std::cout << "Not enough space"; exit(-1);}

	node_max = nodes + num_nodes;
	last_arc = arcs;
	num_arcs = 0;
	
	

	memset(nodes, 0, num_nodes * sizeof(node));


	active_step = 0;
}

Graph::~Graph()
{
	free(nodes);
	free(arcs);
}

void Graph::add_arc_pair(int node_p, int node_q, int cap, int rev_cap)
{
	if (num_arcs_max < num_arcs + 2) { std::cout << "not enough edge space alloacted" << std::endl; exit(-32); }
	
	num_arcs += 2;
	node* p = nodes + node_p;
	node* q = nodes + node_q;

	arc* fow_arc = last_arc++;
	arc* rev_arc = last_arc++;

	fow_arc->next = p->first;
	p->first = fow_arc; 
	
	rev_arc->next = q->first;
	q->first = rev_arc;

	fow_arc->head = q;
	rev_arc->head = p;

	fow_arc->sister = rev_arc;
	rev_arc->sister = fow_arc;

	fow_arc->cap = cap;
	rev_arc->cap = rev_cap;

	fow_arc->pflow = 0;
	rev_arc->pflow = 0;
}

void Graph::add_terminal_cap(int node_p, bool source, int cap)
{
	if (node_p < 0 || node_p >= num_nodes) { std::cout << "Out of bound (add_terminal_cap)" << std::endl; exit(-13); }
	node* p = nodes + node_p;
	p->is_connected_to_source = source;
	p->terminal_cap = cap;
	p->terminal_flow = 0;
}


int Graph::max_flow()
{
	int flow = 0;
	node* augment_node = find_augment();
	while (augment_node) {
		flow += augment(augment_node);
		augment_node = find_augment();
	}
	return flow;
}

Graph::node* Graph::find_augment()
{
	active_step++;
	node *node_p, *node_q;
	arc *curr_arc;
	std::queue< node* > bfs_queue;
	
	for (node* node_p = nodes; node_p != node_max; node_p++) {
		if (node_p->is_connected_to_source && node_p->terminal_cap - node_p->terminal_flow > 0) {
			node_p->parent = nullptr;
			node_p->active = active_step;
			bfs_queue.push(node_p);
		}
	}
	while (!bfs_queue.empty()) {
		node_p = bfs_queue.front();
		bfs_queue.pop();
		if (!node_p->is_connected_to_source && node_p->terminal_cap - node_p->terminal_flow > 0) {
			//connected to sink and positive residual, we found our augmenting path
			return node_p;
		}

		for (curr_arc = node_p->first; curr_arc; curr_arc = curr_arc->next) {
			node_q = curr_arc->head;
			if (node_q->active != active_step) {
				if (curr_arc->cap - curr_arc->pflow > 0) {
					node_q->parent = curr_arc;
					node_q->active = active_step;
					bfs_queue.push(node_q);
				}
			}
		}
	}

	return nullptr; // no path to sink, no augmenting path
}

int Graph::augment(node* node_p)
{
	//compute bottleneck
	node* node_q = node_p;
	int bottleneck = node_q->terminal_cap - node_q->terminal_flow;
	arc* parent_arc = node_q->parent;
	while (parent_arc)
	{
		if (bottleneck > parent_arc->cap - parent_arc->pflow)
			bottleneck = parent_arc->cap - parent_arc->pflow;
		node_q = parent_arc->sister->head;
		parent_arc = node_q->parent;
	}
	if (bottleneck > node_q->terminal_cap - node_q->terminal_flow) {
		bottleneck = node_q->terminal_cap - node_q->terminal_flow;
	}

	// augment path
	node_q = node_p;
	node_q->terminal_flow += bottleneck;
	parent_arc = node_q->parent;
	while (parent_arc)
	{
		parent_arc->pflow += bottleneck;
		parent_arc->sister->pflow -= bottleneck;

		node_q = parent_arc->sister->head;
		parent_arc = node_q->parent;
	}
	node_q->terminal_flow += bottleneck;
	

	return bottleneck;
}

bool Graph::get_pflow(int node_p, int node_q, int& pflow)
{
	// if the nodes are connected, set pflow to the pseudo flow from p -> q (flow out of p to q minus flow from q to p), else return false
	node* p = nodes + node_p;
	node* q = nodes + node_q;

	for (arc* a = p->first; a; a = a->next) {
		if (a->head == q) {
			pflow = a->pflow;
			return true;
		}
	}
	return false;
}

bool Graph::has_nonsat_path_to_source(int node_p)
{
	node* p = nodes + node_p;
	return p->active == active_step; // last find augment has marked them with the last step taken
}
