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
	
	pos2nodeIndex.reserve(rows);
	int node_id = 0;
	for (int row = 0; row < rows; row++) {
		pos2nodeIndex.push_back(std::vector<int>());
		pos2nodeIndex[row].reserve(columns);
		for (int column = 0; column < columns; column++) {
			pos2nodeIndex[row].push_back(node_id);
			node_id++;
		}
	}
	//std::default_random_engine generator;
	//std::uniform_int_distribution<int> distribution(0, number_of_labels-1);

	//init primals (x) and duals (y) (could init random, here we init with lowest singleton val)
	g = nullptr;
	//optimize_step = 0;

	x.reserve(rows);
	//y.reserve(rows);
	for (int row = 0; row < rows; row++) {

		x.push_back(std::vector< int >());
		x[row].reserve(columns);

		//y.push_back(std::vector< std::vector<FourNeighboors> >());
		//y[row].reserve(columns);


		for (int column = 0; column < columns; column++) {
			x[row].push_back(image.at<uchar>(row, column)); //x_p, 

			//y[row].push_back(std::vector<FourNeighboors>());
			//y[row][column].reserve(number_of_labels);
			//for (int c = 0; c < number_of_labels; c++)
			//{
			//	y[row][column].push_back({ 0,0 }); // yp_right(c), yp_down(c), but also by conjuguacy -yleft_p(c), -yup_p(c) respectively
			//}
		}
	}

	y.reserve(number_of_labels);
	for (int c = 0; c < number_of_labels; c++)
	{
		y.push_back(std::vector< std::vector<FourNeighboors> >());
		y[c].reserve(rows);
		for (int row = 0; row < rows; row++) {
			y[c].push_back(std::vector<FourNeighboors>());
			y[c][row].reserve(columns);
			for (int column = 0; column < columns; column++) {
				y[c][row].push_back({ 0,0 });
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
				y[x_p][row][column].right = distance(x_p, x_q);
			}

			if (row != rows - 1) { // there is a down node
				x_q = x[row + 1][column];
				y[x_p][row][column].down = distance(x_p, x_q); //from 0 to wpq dist(x_p,x_q)
			}

		}
	}

}

void GridPrimalDual::printPrimalDual()
{
	for (auto& row : x)
		for (auto& vertex : row)
			std::cout << vertex << std::endl;

}

int GridPrimalDual::distance(const int a, const int b)
{
	if (use_squared)
		return distance_multiplier * truncatedSquaredDifference(a, b, 1, dist_trunc);
	else
		return distance_multiplier * truncatedAbsoluteDifference(a, b, 1, dist_trunc);
}

int GridPrimalDual::singletonPotential(const int row, const int column, int c)
{
	if (use_squared)
		return truncatedSquaredDifference(image.at<uchar>(row, column), c, 1, singleton_trunc);
	else
		return truncatedAbsoluteDifference(image.at<uchar>(row, column), c, 1, singleton_trunc);
}

void GridPrimalDual::optimize()
{
	g = new GraphTree(/*estimated # of nodes*/ rows * columns, /*estimated # of edges*/ 4 * rows * columns);
	std::vector < std::vector< int >> x_old;
	for (int row = 0; row < rows; row++)
	{
		x_old.push_back(std::vector<int>());
		for (int column = 0; column < columns; column++)
		{
			x_old[row].push_back(x[row][column]);
		}
	}
	int loop_count = 0;
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
		std::cout << "loop n " << loop_count << std::endl;
		loop_count++;
		
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
				int y_pq_c = y[c][row][column].right;
				int y_qp_xq = -y[x_q][row][column].right;
				int y_pq_xp = y[x_p][row][column].right;
				int y_qp_c = -y[c][row][column].right;


				if ((y_pq_c + y_qp_xq > distance(c, x_q)) || (y_pq_xp + y_qp_c > distance(x_p, c))) {
					y[c][row][column].right = distance(c, x_q) - y_qp_xq;
				}
			}

			if (row != rows - 1) { // there is a down node
				x_q = x[row + 1][column];
				int y_pq_c = y[c][row][column].down;
				int y_qp_xq = -y[x_q][row][column].down;
				int y_pq_xp = y[x_p][row][column].down;
				int y_qp_c = -y[c][row][column].down;


				if ((y_pq_c + y_qp_xq > distance(c, x_q)) || (y_pq_xp + y_qp_c > distance(x_p, c))) {
					y[c][row][column].down = distance(c, x_q) - y_qp_xq;
				}
			}
		}
	}
}

void GridPrimalDual::updateDualsPrimals(int c)
{
	// construct Graph (Capacities and Flow)

	//std::cout << "constructing the graph " << c << std::endl;
	g->reset();
	int node_id, s_link = 0, diff_height;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			node_id = pos2nodeIndex[row][column];
			//printf("(%d, %d)", std::max(label_height(row, column, x[row][column]) - label_height(row, column, c), 0), std::max(label_height(row, column, c) - label_height(row, column, x[row][column]), 0));
			diff_height = label_height_diff(row, column, x[row][column], c);//  label_height(row, column, x[row][column]) - label_height(row, column, c);
			if (diff_height > 0) {
				g->add_terminal_cap(
					node_id,
					true,
					diff_height);
				s_link++;
			}
			else {
				g->add_terminal_cap(
					node_id,
					false,
					-diff_height);
			}
		}
		//printf("\n");
	}
	printf("s link = %d\n", s_link);
	if (s_link == 0) return;
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
						std::max(distance(c, x_q) - (y[x_q][row - 1][column].down - y[c][row - 1][column].down), 0), // y[x_q][row - 1][column].down - y[c][row - 1][column].down
						std::max(distance(x_p, c) - (y[c][row - 1][column].down - y[x_p][row - 1][column].down), 0)); // y[c][row-1][column].down - y[x_p][row-1][column].down
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
						std::max(distance(c, x_q) - (y[x_q][row][column - 1].right - y[c][row][column - 1].right), 0), // y[x_q][row][column - 1].right - y[c][row][column - 1].right
						std::max(distance(x_p, c) - (y[c][row][column - 1].right - y[x_p][row][column - 1].right), 0)); //  y[c][row][column-1].right - y[x_p][row][column-1].right
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
	flow = g->max_flow();
	printf("Flow = %d\n", flow);
	if (flow == 0)
		return;
	//update duals
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			int node_p = pos2nodeIndex[row][column];
			if (row < rows - 1) { // there is a down node
				int node_q = pos2nodeIndex[row + 1][column];
				//std::cout << node_p << " " << node_q << std::endl;
				int pflow;
				if (g->get_pflow(node_p, node_q, pflow)) {
					y[c][row][column].down += pflow;
				}
				else {
					std::cout << "no such nodes, problem with the code";
					exit(-1);
				}

			}

			if (column < columns - 1) { // there is a right node
				int node_q = pos2nodeIndex[row][column + 1];
				//std::cout << node_p << " " << node_q << std::endl;
				int pflow;
				if (g->get_pflow(node_p, node_q, pflow)) {
					y[c][row][column].right += pflow;
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
			if (g->has_nonsat_path_to_source(pos2nodeIndex[row][column])) {
				if (column != columns - 1) { // there is a right node
					x_q = x[row][column + 1];

					int y_pq_xp = y[x_p][row][column].right;
					int y_qp_xq = -y[x_q][row][column].right;


					if (y_pq_xp + y_qp_xq > distance(x_p, x_q)) {
						y[c][row][column].right = distance(x_p, x_q) + y[x_q][row][column].right;	
					}
				}

				if (row != rows - 1) { // there is a down node
					x_q = x[row + 1][column];
					int y_pq_xp = y[x_p][row][column].down;
					int y_qp_xq = -y[x_q][row][column].down;


					if (y_pq_xp + y_qp_xq > distance(x_p, x_q)) {
						// either x_p=c or x_q=c
						y[c][row][column].down = distance(x_p, x_q) + y[x_q][row][column].down;
						
					}
				}
			}
		}
	}
}


int GridPrimalDual::truncatedSquaredDifference(const int a, const int b, const int kappa, const int truncation)
{
	// Truncated quadratic
	int dist = a - b;
	return MIN(kappa * dist * dist, truncation);
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
		ret -= y[c][row - 1][column].down;
	}
	//if (row != rows - 1) { // has down
	ret += y[c][row][column].down;
	//}
	if (column != 0) { // has left
		ret -= y[c][row][column - 1].right;
	}
	//if (column != columns - 1) { // has right
	ret += y[c][row][column].right;
	//}

	return ret;
}

int GridPrimalDual::label_height_diff(int row, int column, int a, int b)
{
	int ret = singletonPotential(row, column, a) - singletonPotential(row, column, b);
	if (row != 0) { // has up
		ret -= y[a][row - 1][column].down - y[b][row - 1][column].down;
	}
	//if (row != rows - 1) { // has down
	ret += y[a][row][column].down - y[b][row][column].down;
	//}
	if (column != 0) { // has left
		ret -= y[a][row][column - 1].right - y[b][row][column - 1].right;
	}
	//if (column != columns - 1) { // has right
	ret += y[a][row][column].right - y[b][row][column].right;
	//}

	return ret;
}

int GridPrimalDual::load(int prow, int pcol, int qrow, int qcol, int a, int b)
{
	if (pcol == qcol) {
		if (prow + 1 == qrow) {
			//down
			return y[a][prow][pcol].down - y[b][prow][pcol].down;
		}
		else if (prow - 1 == qrow) {
			//up
			return y[b][qrow][qcol].down - y[a][qrow][qcol].down;
		}
		else { throw "not adjacent"; }
	}
	else if (prow == qrow) {
		if (pcol + 1 == qcol) {
			// right
			return y[a][prow][pcol].right - y[b][prow][pcol].right;
		}
		else if (pcol - 1 == qcol) {
			// left
			return y[b][qrow][qcol].right - y[a][qrow][qcol].right;
		}
		else { throw "not adjacent"; }
	}
	throw "not adjacent";
}
