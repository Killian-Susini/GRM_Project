#include "GraphTree.h"

/*
	special constants for node->parent, inspired by the one defined by kolmogorov algorithm
*/
#define SOURCE ( (arc *) 1 )		/* to source */
#define ORPHAN   ( (arc *) 2 )		/* orphan */


GraphTree::GraphTree(int _num_nodes, int _num_arcs_max)
{
	num_nodes = _num_nodes;
	num_arcs_max = _num_arcs_max;
	nodes = (node*)malloc(num_nodes * sizeof(node));
	arcs = (arc*)malloc(num_arcs_max * sizeof(arc));
	if (!nodes || !arcs) { std::cout << "Not enough space"; exit(-1); }
	node_max = nodes + num_nodes;
	last_arc = arcs;
	num_arcs = 0;



	memset(nodes, 0, num_nodes * sizeof(node));


	time_step = 0;
}

GraphTree::~GraphTree()
{
	free(nodes);
	free(arcs);
}

void GraphTree::add_arc_pair(int node_p, int node_q, int cap, int rev_cap)
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

void GraphTree::add_terminal_cap(int node_p, bool source, int cap)
{
	if (node_p < 0 || node_p >= num_nodes) { std::cout << "Out of bound (add_terminal_cap)" << std::endl; exit(-13); }
	node* p = nodes + node_p;
	p->is_connected_to_source = source;
	p->parent = nullptr;
	p->terminal_cap = cap;
	p->terminal_flow = 0;
}

void GraphTree::reset()
{
	last_arc = arcs;
	num_arcs = 0;
	time_step = 0;
	memset(nodes, 0, num_nodes * sizeof(node));
}

bool GraphTree::get_pflow(int node_p, int node_q, int& pflow)
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

bool GraphTree::has_run()
{
	return time_step != 0;
}

bool GraphTree::has_nonsat_path_to_source(int node_p)
{
	//if time_step isn't 0
	node* p = nodes + node_p;
	return p->time_step == time_step;
}

int GraphTree::max_flow()
{
	//active_nodes = std::queue<node*>();
	time_step = 0;
	int flow = 0;
	for (node* u = nodes; u != node_max; u++)
	{
		if (u->is_connected_to_source && u->terminal_cap - u->terminal_flow > 0) {
			u->parent = SOURCE;
			u->in_source_tree = true;
			active_nodes.push(u);
		}
	}
	node* p;
	while (true)
	{
		time_step++;
		p = growth();
		if (!p) { break; }
		flow += augment(p);
		adopt();
	}
	node* u;
	/*int sflow = 0, tflow = 0, s_sat = 0;
	
	for (u = nodes; u != node_max; u++)
	{
		if (u->is_connected_to_source) {
			if (u->terminal_cap - u->terminal_flow == 0) {
				s_sat++;
			}
			sflow += u->terminal_flow;

		}
		else {
			tflow += u->terminal_flow;
		}

	}
	std::cout << "num_sat " << s_sat << "s and t " << sflow << " " << tflow << std::endl;*/

	// mark nodes that are reachable
	std::stack<node*> node_stack;
	
	time_step = -1;
	for (u = nodes; u != node_max; u++)
	{
		if (u->is_connected_to_source && u->terminal_cap - u->terminal_flow > 0) {
			u->time_step = time_step; //overloaded active, GridPrimalDual will use that to know which nodes is recheable (and therefore which x will have their labels changed)
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
			if (v->time_step != time_step && uv->cap - uv->pflow > 0) {
				v->time_step = time_step;
				node_stack.push(v);
			}
		}
	}


	return flow;
}

GraphTree::node* GraphTree::growth()
{
	while (!active_nodes.empty())
	{
		node* p = active_nodes.front();
		active_nodes.pop();
		if (!p->parent) { // not an actual active node, was added in adopt but did not find a parent
			continue;
		}

		if (!p->is_connected_to_source && p->terminal_cap - p->terminal_flow > 0) {
			//found a path
			active_nodes.push(p);
			return p;
		}

		for (arc* pq = p->first; pq; pq = pq->next)
		{
			node* q = pq->head;
			if (!q->in_source_tree && pq->cap - pq->pflow > 0) {
				active_nodes.push(q);
				q->in_source_tree = true;
				q->parent = pq->sister; //points to p
			}
		}
	}
	return nullptr;
}

int GraphTree::augment(node* node_p)
{
	//orphan_nodes = std::queue<node*>();
	//compute bottleneck
	node* q = node_p;
	int bottleneck = q->terminal_cap - q->terminal_flow;
	arc* parent_arc = q->parent;
	if (parent_arc == SOURCE) {
		std::cout << "impossible? " << std::endl;
	}
	while (parent_arc != SOURCE)
	{
		if (bottleneck > parent_arc->sister->cap - parent_arc->sister->pflow)
			bottleneck = parent_arc->sister->cap - parent_arc->sister->pflow;
		q = parent_arc->head;
		parent_arc = q->parent;
	}
	if (bottleneck > q->terminal_cap - q->terminal_flow) {
		bottleneck = q->terminal_cap - q->terminal_flow;
	}

	// augment path
	q = node_p;
	q->terminal_flow += bottleneck;
	parent_arc = q->parent;
	while (parent_arc != SOURCE)
	{
		parent_arc->pflow -= bottleneck;
		parent_arc->sister->pflow += bottleneck;

		if (parent_arc->sister->cap - parent_arc->sister->pflow == 0) {
			q->parent = ORPHAN;
			orphan_nodes.push(q);
		}

		q = parent_arc->head;
		parent_arc = q->parent;
	}
	q->terminal_flow += bottleneck;
	if (q->terminal_cap - q->terminal_flow == 0) {
		q->parent = ORPHAN;
		orphan_nodes.push(q);
	}
	
	return bottleneck;
}

void GraphTree::adopt()
{ //applying (and partially inspired from) the implementation of the paper mentioned by the fast-PD paper 
	while (!orphan_nodes.empty()) {
		node* q = orphan_nodes.front();
		orphan_nodes.pop();
		//process
		int dist, dist_best = INT_MAX;
		arc* best_parent = nullptr;
		for (arc* qp = q->first; qp; qp = qp->next) {
			//search for a parent
			if (qp->sister->cap - qp->sister->pflow > 0) {
				node* j = qp->head;
				if (j->in_source_tree && j->parent) {
					//checking if it is truly connected to the source, and compute the distance to source
					dist = 0;
					while (true) {
						if (j->time_step == time_step) {
							//already did the job, we can break
							dist += j->distance_from_source;
							break;
						}
						// otherwise, we go up and will update the dist of all the node in the pass
						dist++;
						if (j->parent == SOURCE) {
							j->time_step = time_step;
							j->distance_from_source = 1;
							break;
						} 
						if (j->parent == ORPHAN) {
							dist = INT_MAX;
							break;
						}
						j = j->parent->head;
					}
					if (dist < INT_MAX) {
						// j is attached to source
						if (dist < dist_best) { //update best
							dist_best = dist;
							best_parent = qp;
						}
						// set the dist along the path
						for (j = qp->head; j->time_step != time_step; j = j->parent->head) {
							j->time_step = time_step;
							j->distance_from_source = dist--;
						}
					}
				}
			}
		}
		q->parent = best_parent;
		if (q->parent != nullptr) {
			//found one
			q->time_step = time_step;
			q->distance_from_source = dist_best + 1;
		}
		else {
			q->in_source_tree = false;
			//no parent found, we process its children (by processing its neighboor and checking if they have the node as parent)
			for (arc* qp = q->first; qp; qp = qp->next) {
				node* j = qp->head;
				if (j->in_source_tree && j->parent) {
					if (qp->sister->cap - qp->sister->pflow > 0) {
						// if j's parent is q, then it will either find a parent from source, in which case it will have a free node neighboor (q)
						//					   otherwise, it will not have a parent setup, and be discarded from the queue automatically
						// else, then it is still true that it's neighbor has a free node and is from the source (although then it should have setup as a valid parent for q.
						active_nodes.push(j);
					}
					if (j->parent != SOURCE && j->parent != ORPHAN && j->parent->head == q) {
						j->parent = ORPHAN;
						orphan_nodes.push(j);
					}
				}
			}
		}
	}
}

