#include "PrimalDualMRF.h"
#include <opencv2/opencv.hpp>
PrimalDualMRF::PrimalDualMRF(cv::Mat _image, int _number_of_labels, function<int(int, int)> _distanceFunction, function<int(int, int)> _singletonDistanceFunction)
{
	image = _image;
	number_of_labels = _number_of_labels;
	distanceFunction = _distanceFunction;
	singletonDistanceFunction = _singletonDistanceFunction;
	rows = (int)image.rows;
	cols = (int)image.cols;
	number_of_labels = _number_of_labels;

	num_nodes = rows*cols;
	num_arcs_max = rows * cols * 4;
	if (num_nodes == 0) {
		return;
	}
	x = (node*)malloc(num_nodes * sizeof(node));
	y = (arc*)malloc(num_arcs_max * sizeof(arc));
	if (!x || !y) { std::cout << "Not enough space"; exit(-1); }
	x_max = x + num_nodes;
	last_arc = y;
	num_arcs = 0;

	memset(x, 0, num_nodes * sizeof(node));

	g = nullptr;
	//init primals (x) and duals (y) (could init random, here we init with lowest singleton val)
	//optimize_step = 0;
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			x[row * cols + col].first = NULL;
			x[row * cols + col].x = image.at<uchar>(row, col);			 //x_p, 
			x[row * cols + col].x_old_label = image.at<uchar>(row, col);
			x[row * cols + col].id = row * cols + col;
		}
	}
	node* p, * q;
	//arc* pq;
	for (int row = 0; row < rows-1; row++) {
		for (int col = 0; col < cols-1; col++) {
			p = x + row * cols + col;
			add_arcs_pair(row * cols + col, row * cols + col + 1);
			q = x + row * cols + col + 1;
			//adjust ypq(xp) so that load_pq(xp,xq)=ypq(xp)+yqp(xq)=wpq dist(xp,xq)
			p->first->y[p->x] = distanceFunction(p->x, q->x);
			p->first->sister->y[p->x] = -distanceFunction(p->x, q->x);


			add_arcs_pair(row * cols + col, (row+1) * cols + col);
			q = x + (row + 1) * cols + col;
			//adjust ypq(xp) so that load_pq(xp,xq)=ypq(xp)+yqp(xq)=wpq dist(xp,xq)
			p->first->y[p->x] = distanceFunction(p->x, q->x);
			p->first->sister->y[p->x] = -distanceFunction(p->x, q->x);


		}
	}
}

PrimalDualMRF::~PrimalDualMRF()
{
	free(x);
	for (arc* a = y; a != last_arc; a++) {
		free(a->y);
	}
	free(y);
}

inline
void PrimalDualMRF::add_arcs_pair(int node_p, int node_q)
{
	if (num_arcs_max < num_arcs + 2) { std::cout << "not enough edge space alloacted" << std::endl; exit(-32); }

	num_arcs += 2;
	node* p = x + node_p;
	node* q = x + node_q;
	arc *fow_arc, *rev_arc;
	
	fow_arc = last_arc++;
	rev_arc = last_arc++;
	fow_arc->next = p->first;
	p->first = fow_arc;

	rev_arc->next = q->first;
	q->first = rev_arc;

	fow_arc->head = q;
	rev_arc->head = p;

	fow_arc->sister = rev_arc;
	rev_arc->sister = fow_arc;

	fow_arc->y = (int*)malloc(number_of_labels * sizeof(int));
	rev_arc->y = (int*)malloc(number_of_labels * sizeof(int));
	for (int c = 0; c < number_of_labels; c++) {
		fow_arc->y[c] = 0;
		rev_arc->y[c] = 0;
	}
}

void PrimalDualMRF::optimize()
{
	g = new Graph(/*estimated # of nodes*/ rows * cols, /*estimated # of edges*/ 4 * rows * cols);
	std::vector < std::vector< int >> x_old;
	for (int row = 0; row < rows; row++)
	{
		x_old.push_back(std::vector<int>());
		for (int col = 0; col < cols; col++)
		{
			x_old[row].push_back(x[row*cols + col].x);
		}
	}
	int loop_count = 0;
	bool cont = true;
	while (cont) {
		for (int row = 0; row < rows; row++)
		{
			for (int col = 0; col < cols; col++)
			{
				x_old[row][col] = x[row * cols + col].x;
			}
		}
		cont = false;

		for (int c = 0; c < number_of_labels; c++) {
			preEditDuals(c);
			//std::cout << "done pre-edit n" << c << std::endl;
			updateDualsPrimals(c);
			//std::cout << "done duals and primals update n" << c << std::endl;
			postEditDuals(c);
			//std::cout << "done post-edit n" << c << std::endl;
		}
		for (int row = 0; row < rows; row++)
		{
			for (int col= 0; col < cols; col++)
			{
				if (x_old[row][col] != x[row * cols + col].x) {
					cont = true;
				}
			}
		}
		//optimize_step++;
		std::cout << "loop n " << loop_count << std::endl;
		loop_count++;

	}

	delete g;
}

void PrimalDualMRF::preEditDuals(int c)
{
	node *p;
	arc* pq;
	int y_pq_c, y_qp_xq, y_pq_xp, y_qp_c, tmp_dist;
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			p = x + row*cols + col;
			for (pq = p->first; pq; pq = pq->next) {
				y_pq_c = pq->y[c];
				y_qp_xq = -pq->y[pq->head->x];
				y_pq_xp = pq->y[p->x];
				y_qp_c = -y_pq_c;
				tmp_dist = distanceFunction(c, pq->head->x);
				if ((y_pq_c + y_qp_xq > tmp_dist) || (y_pq_xp + y_qp_c > distanceFunction(p->x, c))) {
					pq->y[c] = tmp_dist - y_qp_xq;
					pq->sister->y[c] = -pq->y[c];
				}
			}
		}
	}
}

void PrimalDualMRF::updateDualsPrimals(int c)
{
	g->reset();
	node* p;
	int s_link = 0, diff_height;

	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			int node_p = row * cols + col;
			p = x + node_p;
			diff_height = label_height(p, p->x) - label_height(p, c);

			//printf(" height xp = %d\n", label_height(p, p->x));
			//printf(" height c = %d\n", label_height(p, c));
			if (diff_height > 0) {
				g->add_terminal_cap(
					node_p,
					true,
					diff_height);
				s_link++;
			}
			else {
				g->add_terminal_cap(
					node_p,
					false,
					-diff_height);
			}
		}
	}
	printf("s link = %d\n", s_link);
	if (s_link == 0) return;
	
	arc* pq;
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			int node_p = row*cols+col;
			p = x + node_p;
			
			for (pq = p->first; pq; pq = pq->next) {
				if (pq->head->id > node_p) {
					if (p->x != c && pq->head->x != c) {
						g->add_arc_pair(node_p, pq->head->id,
							std::max(distanceFunction(c, pq->head->x) - load(pq, c, pq->head->x), 0),
							std::max(distanceFunction(p->x, c) - load(pq, p->x, c), 0));
					}
					else {
						g->add_arc_pair(node_p, pq->head->id, 0, 0);
					}
				}
			}
						
		}
	}
	int flow;

	flow = g->max_flow_push_relabel();

	if (flow == 0)
		return;
	//printf("flow = %d\n", flow);
	//update duals
	int pflow;
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			int node_p = row * cols + col;
			p = x + node_p;
			for (pq = p->first; pq; pq = pq->next) {
				if ( g->get_pflow(node_p, pq->head->id, pflow)) { // maybe directly link to arc
					pq->y[c] += pflow;
				}
			}
		}
	}


	//update primals
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			if (g->has_nonsat_path_to_source(row*cols+col)) {
				x[row * cols + col].x = c;
			}
		}
	}


}

void PrimalDualMRF::postEditDuals(int c)
{
	node* p;
	arc* pq;
	int y_qp_xq, y_pq_xp, tmp_dist;
	for (int row = 0; row < rows; row++) {
		for (int col = 0; col < cols; col++) {
			p = x + row * cols + col;
			for (pq = p->first; pq; pq = pq->next) {
				y_qp_xq = -pq->y[pq->head->x];
				y_pq_xp = pq->y[p->x];
				tmp_dist = distanceFunction(p->x, pq->head->x);
				if (y_pq_xp + y_qp_xq > tmp_dist) {
					if (p->x == c) 
					{

						pq->y[c] = tmp_dist - y_qp_xq;
						pq->sister->y[c] = -pq->y[c];
					}
					else
					{
						pq->y[c] = tmp_dist - y_pq_xp;
						pq->sister->y[c] = -pq->y[c];
					}
				}
			}
		}
	}
}

inline
int PrimalDualMRF::label_height(node* p, int c)
{
	int ret_val = singletonDistanceFunction(c, p->x_old_label);
	for (arc* pq = p->first; pq; pq = pq->next) {
		ret_val += pq->y[c];
	}
	return ret_val;
}

inline
int PrimalDualMRF::load(arc* pq, int a, int b)
{
	return pq->y[a] - pq->y[b];
}

