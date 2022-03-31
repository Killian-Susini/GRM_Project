#include "GridPrimalDual.h"

//#include <random>

GridPrimalDual::GridPrimalDual(cv::Mat _image, int _number_of_labels, int _distance_multiplier, int _dist_trunc, int _singleton_trunc, bool _use_squared)
{
	image = _image;
	rows = (int)image.rows;
	columns = (int)image.cols;
	number_of_labels = _number_of_labels;
	distance_multiplier = _distance_multiplier;
	dist_trunc = _dist_trunc;
	singleton_trunc = _singleton_trunc;
	use_squared = _use_squared;
	//std::default_random_engine generator;
	//std::uniform_int_distribution<int> distribution(0, number_of_labels-1);

	//init primals (x) and duals (y) (could init random, here we init with lowest singleton val)
	g = nullptr;
	//optimize_step = 0;

	x.reserve(rows);
	y.reserve(rows);
	for (size_t row = 0; row < rows; row++) {

		x.push_back(std::vector< int >());
		x[row].reserve(columns);

		y.push_back(std::vector< std::vector<FourNeighboors> >());
		y[row].reserve(columns);


		for (size_t column = 0; column < columns; column++) {
			x[row].push_back(image.at<uchar>(row,column)); //x_p, 
			
			y[row].push_back(std::vector<FourNeighboors>());
			y[row][column].reserve(number_of_labels);
			for (size_t c = 0; c < number_of_labels; c++)
			{
				y[row][column].push_back({ 0,0 }); // yp_right(c), yp_down(c), but also by conjuguacy -yleft_p(c), -yup_p(c) respectively
			}
		}
	}

	//adjust ypq(xp) so that load_pq(xp,xq)=ypq(xp)+yqp(xq)=wpq dist(xp,xq) (wpq=2)
	int x_p, x_q;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			
			x_p = x[row][column];

			if (column != columns - 1) { // there is a right node
				x_q = x[row][column + 1];
				y[row][column][x_p].right = distance(x_p, x_q);
			}

			if (row != rows - 1) { // there is a down node
				x_q = x[row + 1][column];
				y[row][column][x_p].down = distance(x_p, x_q); //from 0 to wpq dist(x_p,x_q)
			}

		}
	}

}

void GridPrimalDual::printPrimalDual()
{
	for (auto row : x)
		for (auto vertex : row)
			std::cout << vertex << std::endl;

}
int GridPrimalDual::distance(int a, int b)
{
	if (use_squared)
		return distance_multiplier*truncatedSquaredDifference(a,b,1,dist_trunc);
	else
		return distance_multiplier * truncatedAbsoluteDifference(a, b, 1, dist_trunc);
}

int GridPrimalDual::singletonPotential(int row, int column, int c)
{
	if (use_squared)
		return truncatedSquaredDifference(image.at<uchar>(row,column), c, 1, singleton_trunc);
	else
		return truncatedAbsoluteDifference(image.at<uchar>(row, column), c, 1, singleton_trunc);
}

void GridPrimalDual::optimize()
{
	g = new Graph(/*estimated # of nodes*/ rows * columns, /*estimated # of edges*/ 4 * rows * columns);
	std::vector < std::vector< int >> x_old;
	for (int row = 0; row < rows; row++)
	{
		x_old.push_back(std::vector<int>());
		for (int column = 0; column < columns; column++)
		{
			x_old[row].push_back(x[row][column]);
		}
	}
	bool cont = true;
	while (cont) {
		for (int row = 0; row < rows; row++)
		{
			for (int column = 0; column < columns; column++)
			{
				x_old[row][column] = x[row][column];
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
			for (int column = 0; column < columns; column++)
			{
				if (x_old[row][column] != x[row][column]) {
					cont = true;
				}
			}
		}
		//optimize_step++;
	}

	delete g;
}

void GridPrimalDual::preEditDuals(int c)
{
	int x_p, x_q;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			x_p = x[row][column];

			if (column != columns - 1) { // there is a right node
				x_q = x[row][column + 1];
				int y_pq_c = y[row][column][c].right;
				int y_qp_xq = -y[row][column][x_q].right;
				int y_pq_xp = y[row][column][x_p].right;
				int y_qp_c = -y[row][column][c].right;


				if ((y_pq_c + y_qp_xq > distance(c, x_q)) || (y_pq_xp + y_qp_c > distance(x_p, c))) {
					y[row][column][c].right = distance(c, x_q) + y[row][column][x_q].right;
				}
			}

			if (row != rows - 1) { // there is a down node
				x_q = x[row + 1][column];
				int y_pq_c = y[row][column][c].down;
				int y_qp_xq = -y[row][column][x_q].down;
				int y_pq_xp = y[row][column][x_p].down;
				int y_qp_c = -y[row][column][c].down;
				

				if ((y_pq_c + y_qp_xq > distance(c, x_q)) || (y_pq_xp + y_qp_c > distance(x_p, c))) {
					y[row][column][c].down = distance(c, x_q) + y[row][column][x_q].down;
				}
			}
		}
	}
}

void GridPrimalDual::updateDualsPrimals(int c)
{
	// construct Graph (Capacities and Flow)

	std::cout << "constructing the graph " << c << std::endl;
	std::vector<std::vector<int>> pos2nodeIndex;

	pos2nodeIndex.reserve(rows);


	g->reset(); 
	int node_id = 0;
	for (int row = 0; row < rows; row++) {
		pos2nodeIndex.push_back(std::vector<int>());
		pos2nodeIndex[row].reserve(columns);
		for (int column = 0; column < columns; column++) {

			pos2nodeIndex[row].push_back(node_id);
			//printf("(%d, %d)", std::max(label_height(row, column, x[row][column]) - label_height(row, column, c), 0), std::max(label_height(row, column, c) - label_height(row, column, x[row][column]), 0));
			if (label_height(row, column, x[row][column]) - label_height(row, column, c) > 0) {
				g->add_terminal_cap(
					node_id,
					true,
					label_height(row, column, x[row][column]) - label_height(row, column, c));
			}
			else {
				g->add_terminal_cap(
					node_id,
					false,
					label_height(row, column, c) - label_height(row, column, x[row][column]));
			}
			node_id++;
		}
		//printf("\n");
	}


	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			int node_p_id = pos2nodeIndex[row][column], node_q_id;
			int x_p = x[row][column];
			int x_q;
			if (row > 0) { // has up
				node_q_id = pos2nodeIndex[row - 1][column];
				x_q = x[row - 1][column];
				if (x_p != c && x_q != c) {
					//printf("(%d, %d)", std::max(distance(c, x_q) - load(row, column, row - 1, column, c, x_q), 0), std::max(distance(x_p, c) - load(row, column, row - 1, column, x_p, c), 0));

					g->add_arc_pair(node_p_id, node_q_id,
						std::max(distance(c, x_q) - load(row, column, row - 1, column, c, x_q), 0),
						std::max(distance(x_p, c) - load(row, column, row - 1, column, x_p, c), 0));
				}
				else {
					g->add_arc_pair(node_p_id, node_q_id, 0, 0);
				}
			}
			if (column > 0) { // has left
				node_q_id = pos2nodeIndex[row][column - 1];
				x_q = x[row][column - 1];
				if (x_p != c && x_q != c) {
					g->add_arc_pair(node_p_id, node_q_id,
						std::max(distance(c, x_q) - load(row, column, row, column - 1, c, x_q), 0),
						std::max(distance(x_p, c) - load(row, column, row, column - 1, x_p, c), 0));
				}
				else {
					g->add_arc_pair(node_p_id, node_q_id, 0, 0);
				}
			}
		}
	}
	int flow;

	//if (optimize_step >3) 
	//	flow = g->max_flow();
	//else
	flow = g->max_flow_push_relabel();
	printf("Flow = %d\n", flow);
	if (flow == 0)
		return;
	//update duals
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			int node_p = pos2nodeIndex[row][column];
			if (row < rows - 1) { // there is a down node
				int node_q = pos2nodeIndex[row+1][column];
				//std::cout << node_p << " " << node_q << std::endl;
				int pflow;
				if (g->get_pflow(node_p, node_q, pflow)){
					y[row][column][c].down += pflow;
				}
				else {
					std::cout << "no such nodes, problem with the code";
					exit(-1);
				}

			}

			if (column < columns - 1) { // there is a right node
				int node_q = pos2nodeIndex[row][column+1];
				//std::cout << node_p << " " << node_q << std::endl;
				int pflow;
				if (g->get_pflow(node_p, node_q, pflow)) {
					y[row][column][c].right += pflow;
				}
				else {
					std::cout << "no such nodes, problem with the code";
					exit(-1);
				}
			}
		}
	}


	//update primals
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			if (g->has_nonsat_path_to_source(pos2nodeIndex[row][column])) {
				x[row][column] = c;
			}
		}
	}



}

void GridPrimalDual::postEditDuals(int c)
//technically only useful if one of p or q was changed during the update
{
	int x_p, x_q;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			x_p = x[row][column];

			if (column != columns - 1) { // there is a right node
				x_q = x[row][column + 1];

				int y_pq_xp = y[row][column][x_p].right;
				int y_qp_xq = -y[row][column][x_q].right;


				if (y_pq_xp + y_qp_xq > distance(x_p, x_q)) {
					// either x_p=c or x_q=c
					if (x_p == c) {
						y[row][column][c].right = distance(x_p, x_q) + y[row][column][x_q].right;
					}
					else {
						y[row][column][c].right = distance(x_p, x_q) - y[row][column][x_p].right;
					}
					
				}
			}

			if (row != rows - 1) { // there is a down node
				x_q = x[row + 1][column];
				int y_pq_xp = y[row][column][x_p].down;
				int y_qp_xq = -y[row][column][x_q].down;


				if (y_pq_xp + y_qp_xq > distance(x_p, x_q)) {
					// either x_p=c or x_q=c
					if (x_p == c) {
						y[row][column][c].down = distance(x_p, x_q) + y[row][column][x_q].down;
					}
					else {
						y[row][column][c].down = distance(x_p, x_q) - y[row][column][x_p].down;
					}

				}
			}
		}
	}
}


int GridPrimalDual::truncatedSquaredDifference(int a, int b, int kappa, int truncation)
{
	// Truncated quadratic
	int dist = a - b;
	return MIN(kappa*dist*dist, truncation);
}

int GridPrimalDual::truncatedAbsoluteDifference(int a, int b, int kappa, int truncation) {
	// Truncated absolute
	int dist = abs(a - b);
	return MIN(kappa * dist, truncation);
}

int GridPrimalDual::label_height(int row, int column, int c)
{
	int ret = singletonPotential(row, column, c);
	if (row != 0) { // has up
		ret -= y[row-1][column][c].down;
	}
	if (row != rows-1) { // has down
		ret += y[row][column][c].down;
	}
	if (column != 0) { // has left
		ret -= y[row][column-1][c].right;
	}
	if (column != columns-1) { // has right
		ret += y[row][column][c].right;
	}

	return ret;
}

int GridPrimalDual::load(int prow, int pcol, int qrow, int qcol, int a, int b)
{
	if (pcol == qcol) {
		if (prow + 1== qrow) {
			//down
			return y[prow][pcol][a].down - y[prow][pcol][b].down;
		}
		else if (prow - 1 == qrow ) {
			//up
			return y[qrow][qcol][b].down - y[qrow][qcol][a].down;
		} 
		else { throw "not adjacent"; }
	}
	else if (prow == qrow) {
		if (pcol + 1 == qcol) {
			// right
			return y[prow][pcol][a].right - y[prow][pcol][b].right;
		}
		else if (pcol - 1 == qcol) {
			// left
			return y[qrow][qcol][b].right - y[qrow][qcol][a].right;
		}
		else { throw "not adjacent"; }
	}
	throw "not adjacent";
}

void GridPrimalDual::recomputeHeights(std::vector<std::vector<int>>& height, std::vector<std::vector<NodesNeighboors>>& C, std::vector<std::vector<NodesNeighboors>>& F)
{
	// FAUXXXXX
	std::queue<std::pair<int, int>> node_queue;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			if ((C[row][column].sink - F[row][column].sink) > 0) {
				height[row][column] = 1;
				node_queue.push(std::pair<int, int>(row, column));
			}
			else
			{
				height[row][column] = rows*columns+3;
			}
		}
	}
	int max_height = 0;
	while (!node_queue.empty()) {
		auto p = node_queue.front();
		node_queue.pop();
		int row = p.first;
		int column = p.second;
		max_height = std::max(max_height, height[row][column]);
		if (row > 0 && height[row - 1][column] == (rows * columns + 3) && (C[row - 1][column].down - F[row - 1][column].down) > 0) {
			height[row - 1][column] = height[row][column] + 1;
			node_queue.push(std::pair<int, int>(row - 1, column));
		}
		if (row < rows - 1 && height[row + 1][column] == (rows * columns + 3) && (C[row + 1][column].up - F[row + 1][column].up) > 0) {
			height[row + 1][column] = height[row][column] + 1;
			node_queue.push(std::pair<int, int>(row + 1, column));
		}
		if (column > 0 && height[row][column - 1] == (rows * columns + 3) && (C[row][column - 1].right - F[row][column - 1].right) > 0) {
			height[row][column - 1] = height[row][column] + 1;
			node_queue.push(std::pair<int, int>(row, column - 1));
		}
		if (column < columns - 1 && height[row][column + 1] == (rows * columns + 3) && (C[row][column + 1].left - F[row][column + 1].left) > 0) {
			height[row][column + 1] = height[row][column] + 1;
			node_queue.push(std::pair<int, int>(row, column + 1));
		}
	}
	//std::cout << "computed heights " << max_height << std::endl;
}


void GridPrimalDual::push2Sink(int prow, int pcol, std::vector<std::vector<int>>& excess, std::vector<std::vector<NodesNeighboors>>& C, std::vector<std::vector<NodesNeighboors>>& F)
{
	int max_flow_possible = std::min(excess[prow][pcol], C[prow][pcol].sink - F[prow][pcol].sink);
	excess[prow][pcol] -= max_flow_possible;
	F[prow][pcol].sink += max_flow_possible;
	//no need flow from sink
}

void GridPrimalDual::pushFromSource(int qrow, int qcol, std::vector<std::vector<int>>& excess, std::vector<std::vector<int>>& source2p_capacity, std::vector<std::vector<int>>& source2p_flow, std::vector<std::vector<NodesNeighboors>>& F)
{
	// excess of source is infinite
	int max_flow_possible = source2p_capacity[qrow][qcol] - source2p_flow[qrow][qcol];
	excess[qrow][qcol] += max_flow_possible;
	source2p_flow[qrow][qcol] += max_flow_possible;
	F[qrow][qcol].source -= max_flow_possible;
}

void GridPrimalDual::push2Source(int prow, int pcol, std::vector<std::vector<int>>& excess, std::vector<std::vector<int>>& source2p_flow, std::vector<std::vector<NodesNeighboors>>& C, std::vector<std::vector<NodesNeighboors>>& F)
{
	// excess of source is infinite
	int max_flow_possible = std::min(excess[prow][pcol], C[prow][pcol].source - F[prow][pcol].source);
	excess[prow][pcol] -= max_flow_possible;
	F[prow][pcol].source += max_flow_possible;
	source2p_flow[prow][pcol] -= max_flow_possible;
}

void GridPrimalDual::relabel(int prow, int pcol, std::vector<std::vector<int>>& height, std::vector<std::vector<NodesNeighboors>>& C, std::vector<std::vector<NodesNeighboors>>& F)
{
	int min_height = INT_MAX;
	const auto& F_p = F[prow][pcol];
	const auto& C_p = C[prow][pcol];
	
	if (C_p.left - F_p.left > 0) { // there is an edge in the residual graph from p to left
		min_height = std::min(min_height, height[prow][pcol-1]);
		height[prow][pcol] = min_height + 1;
	}
	if (C_p.right - F_p.right > 0) { // there is an edge in the residual graph from p to left
		min_height = std::min(min_height, height[prow][pcol+1]);
		height[prow][pcol] = min_height + 1;
	}
	if (C_p.down - F_p.down > 0) { // there is an edge in the residual graph from p to left
		min_height = std::min(min_height, height[prow+1][pcol]);
		height[prow][pcol] = min_height + 1;
	}
	if (C_p.up - F_p.up > 0) { // there is an edge in the residual graph from p to left
		min_height = std::min(min_height, height[prow-1][pcol]);
		height[prow][pcol] = min_height + 1;
	}
	if (C_p.source - F_p.source > 0) { // there is an edge in the residual graph from p to left
		min_height = std::min(min_height, rows*columns + 2); // source height
		height[prow][pcol] = min_height + 1;
	}
}

void GridPrimalDual::discharge(int prow, int pcol, std::vector<std::vector<int>>& excess, std::vector<std::vector<int>>& height, std::vector<std::vector<int>>& source2p_capacity, std::vector<std::vector<int>>& source2p_flow, std::vector<std::vector<NodesNeighboors>>& C, std::vector<std::vector<NodesNeighboors>>& F)
{
	while (excess[prow][pcol] > 0) {
		// go over each neighboor and sink and source, and push if possible, otherwise relabel
		if (C[prow][pcol].sink - F[prow][pcol].sink) { //can always push to sink
			push2Sink(prow, pcol, excess, C, F);
		}
		else if ((prow > 0) && (C[prow][pcol].up - F[prow][pcol].up) && (height[prow][pcol] > height[prow - 1][pcol])) {
			//push up
			int max_possible = std::min(excess[prow][pcol], C[prow][pcol].up - F[prow][pcol].up);
			excess[prow][pcol] -= max_possible;
			excess[prow-1][pcol] += max_possible;
			F[prow][pcol].up += max_possible;
			F[prow-1][pcol].down -= max_possible;
		}
		else if ((prow < rows-1) && (C[prow][pcol].down - F[prow][pcol].down) && (height[prow][pcol] > height[prow + 1][pcol])) {
			//push down
			int max_possible = std::min(excess[prow][pcol], C[prow][pcol].down - F[prow][pcol].down);
			excess[prow][pcol] -= max_possible;
			excess[prow + 1][pcol] += max_possible;
			F[prow][pcol].down += max_possible;
			F[prow + 1][pcol].up -= max_possible;
		}
		else if((pcol > 0) && (C[prow][pcol].left - F[prow][pcol].left) && (height[prow][pcol] > height[prow][pcol - 1])) {
			//push left
			int max_possible = std::min(excess[prow][pcol], C[prow][pcol].left - F[prow][pcol].left);
			excess[prow][pcol] -= max_possible;
			excess[prow][pcol - 1] += max_possible;
			F[prow][pcol].left += max_possible;
			F[prow][pcol - 1].right -= max_possible;
		}
		else if ((pcol < columns - 1) && (C[prow][pcol].right - F[prow][pcol].right) && (height[prow][pcol] > height[prow][pcol + 1])) {
			//push right
			int max_possible = std::min(excess[prow][pcol], C[prow][pcol].right - F[prow][pcol].right);
			excess[prow][pcol] -= max_possible;
			excess[prow][pcol + 1] += max_possible;
			F[prow][pcol].right += max_possible;
			F[prow][pcol + 1].left -= max_possible;
		}
		else if ((C[prow][pcol].source - F[prow][pcol].source) && (height[prow][pcol] > rows*columns + 2)) { //source height is the total number of vertices
			push2Source(prow, pcol, excess, source2p_flow, C, F);
		}
		else {

			//number_of_relabel++;
			//if (number_of_relabel >  rows * columns + 2) {
				// global relabeling heuristic
			//	recomputeHeights(height, C, F);
			//	number_of_relabel = 0;
			//}
			//else {
				//relabel
				
			//	relabel(prow, pcol, height, C, F);
			//}
			relabel(prow, pcol, height, C, F);
		}
	}
}
