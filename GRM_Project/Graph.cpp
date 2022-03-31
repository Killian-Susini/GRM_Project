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

void Graph::reset()
{
	last_arc = arcs;
	num_arcs = 0;
	active_step = 0;
	memset(nodes, 0, num_nodes * sizeof(node));
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



int Graph::max_flow_push_relabel()
{

	active_step = -1; //overloaded from augmenting style implementation to reuse previous code in GridPrimalDual...
	node* u;
	std::list<node*> active_nodes;
	
	for (u = nodes; u != node_max; u++)
	{
		u->active = 0;
		u->label = 0;
		if (u->is_connected_to_source) {
			//push from source all we can
			u->terminal_flow += u->terminal_cap;
			u->active += u->terminal_cap;
		}
		active_nodes.push_back(u);
	}
	
	auto next_active = active_nodes.begin();
	int old_label;
	while (next_active != active_nodes.end())
	{
		old_label = (*next_active)->active;
		discharge(*next_active);
		if ((*next_active)->active > old_label) {
			active_nodes.splice(active_nodes.begin(), active_nodes, next_active);
		}
		else {
			next_active++;
		}
	}

	//compute max flow from pflow to sink
	int flow = 0;
	for (u = nodes; u != node_max; u++)
	{
		if (!u->is_connected_to_source) flow += u->terminal_flow;
	}

	//mark all reachable nodes from source by running dfs
	std::stack<node*> node_stack;
	
	for (u = nodes; u != node_max; u++)
	{
		if (u->is_connected_to_source && u->terminal_cap - u->terminal_flow > 0) {
			u->active = active_step; //overloaded active, GridPrimalDual will use that to know which nodes is recheable (and therefore which x will have their labels changed)
			node_stack.push(u);
		}
	}
	arc* uv;
	node* v;
	while (!node_stack.empty()) {
		u = node_stack.top();
		node_stack.pop();
		for (uv = u->first; uv; uv = uv->next) {
			v = uv->head;
			if (v->active != active_step && uv->cap - uv->pflow > 0) {
				v->active = active_step;
				node_stack.push(v);
			}
		}	
	}
	return flow;
}

void Graph::push(node* u, arc* uv)
{
	int max_pushable_flow = std::min(u->active, uv->cap - uv->pflow);
	
	uv->pflow += max_pushable_flow;
	uv->sister->pflow -= max_pushable_flow;

	u->active -= max_pushable_flow;
	uv->head->active += max_pushable_flow;
}


void Graph::relabel(node* u)
{
	int min_neigh_label = INT_MAX;
	if (u->is_connected_to_source && u->terminal_flow > 0) {
		min_neigh_label = num_nodes;
		u->label = min_neigh_label + 1;
	}
	for (arc* uv = u->first; uv; uv = uv->next) {
		if (uv->cap - uv->pflow > 0 && min_neigh_label > uv->head->label) {
			min_neigh_label = uv->head->label;
			u->label = min_neigh_label + 1;
		}
	}
}


void Graph::discharge(node* u)
{
	arc* uv;
	int old_excess;
	while (u->active)
	{
		old_excess = u->active;
		if (!u->is_connected_to_source && u->terminal_cap - u->terminal_flow > 0) {
			//push to sink
			int max_pushable_flow = std::min(u->active, u->terminal_cap - u->terminal_flow);
			u->terminal_flow += max_pushable_flow;
			u->active -= max_pushable_flow;
		}
		else if (u->label > num_nodes && u->is_connected_to_source && u->terminal_flow > 0) {
			//push (back) to source
			int max_pushable_flow = std::min(u->active, u->terminal_flow);
			u->terminal_flow -= max_pushable_flow;
			u->active -= max_pushable_flow;
		}
		else {
			for (uv = u->first; uv; uv = uv->next) {
				if (u->label > uv->head->label && uv->cap - uv->pflow > 0) {
					push(u, uv);
					if (u->active == 0) break;
				}
			}
		}

		if (old_excess == u->active) {
			relabel(u);
		}
	}
}

