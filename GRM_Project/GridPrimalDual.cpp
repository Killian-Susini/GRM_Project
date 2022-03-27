#include "GridPrimalDual.h"
//#include <random>

GridPrimalDual::GridPrimalDual(cv::Mat _image, int _number_of_labels)
{
	image = _image;
	rows = image.rows;
	columns = image.cols;
	number_of_labels = _number_of_labels;
	//std::default_random_engine generator;
	//std::uniform_int_distribution<int> distribution(0, number_of_labels-1);

	//init primals (x) and duals (y) (could init random, here we init with lowest singleton val)

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
	//wpq = 2
	return 2*truncatedSquaredDifference(a,b,200);
}

int GridPrimalDual::singletonPotential(int row, int column, int c)
{
	return truncatedSquaredDifference(image.at<uchar>(row,column), c, 10000);
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

void GridPrimalDual::updateDualsPrimal(int c)
{
	// construct Graph (Capacities and Flow)

	std::vector<std::vector<int>> excess;
	std::vector<std::vector<int>> height;
	std::vector<std::vector<int>> seen;
	
	std::vector<std::vector<NodesNeighboors>> C; // right down left up source sink
	std::vector<std::vector<NodesNeighboors>> F; // Same
	std::vector<std::vector<int>> source2p_capacity;
	std::vector<std::vector<int>> source2p_flow;

	excess.reserve(rows);
	height.reserve(rows);
	seen.reserve(rows);
	C.reserve(rows);
	F.reserve(rows);
	source2p_capacity.reserve(rows);
	source2p_flow.reserve(rows);
	for (int row = 0; row < rows; row++) {
		excess.push_back(std::vector<int>(columns, 0));
		height.push_back(std::vector<int>(columns, 0));
		seen.push_back(std::vector<int>(columns, 0));
		C.push_back(std::vector<NodesNeighboors>());
		F.push_back(std::vector<NodesNeighboors>());
		source2p_capacity.push_back(std::vector<int>());
		source2p_flow.push_back(std::vector<int>(columns, 0));

		C[row].reserve(columns);
		F[row].reserve(columns);
		source2p_capacity[row].reserve(columns);
		for (int column = 0; column < columns; column++) {
			C[row][column] = { 0,0,0,0,0,0 };
			F[row][column] = { 0,0,0,0,0,0 };

			// capacities are more complex

			//first from/to source/sink
			source2p_capacity[row].push_back(std::max(label_height(row,column,x[row][column]) - label_height(row, column, c), 0));
			C[row][column].sink = std::max(label_height(row, column, c) - label_height(row, column, x[row][column]), 0);
			
			//next for the interior nodes, 
			//it depends on whether one of the involved node is set as c (see fig 6.)
			int x_p = x[row][column];
			if (x_p != c) {
				int x_q;
				if (row != 0) { // has left
					x_q = x[row - 1][column];
					if (x_q != c) {
						C[row][column].left = std::max(distance(c, x_q) - load(row, column, row - 1,column,c, x_q), 0);
					}
				}
				if (row != rows - 1) { // has right
					x_q = x[row + 1][column];
					if (x_q != c) {
						C[row][column].right = std::max(distance(c, x_q) - load(row, column, row + 1, column, c, x_q), 0);
					}
				}
				if (column != 0) { // has up
					x_q = x[row][column - 1];
					if (x_q != c) {
						C[row][column].up = std::max(distance(c, x_q) - load(row, column, row, column - 1, c, x_q), 0);
					}
				}
				if (column != columns - 1) { // has down
					x_q = x[row][column + 1];
					if (x_q != c) {
						C[row][column].down = std::max(distance(c, x_q) - load(row, column, row, column + 1, c, x_q), 0);
					}
				}
			}// else the 4 caps stay at 0
		}
	}

	int height_source = rows * columns;
	int excess_source = INT_MAX;
	int seen_source = 0;

	int height_sink = 0;
	int excess_sink = 0;
	int seen_sink = 0;

	// solve with max flow, keep the flows
	//TODO



	// update duals

	// update primals
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


int GridPrimalDual::truncatedSquaredDifference(int a, int b, int truncation)
{
	// Truncated quadratic
	int dist = a - b;
	return MAX(dist*dist, truncation);
}

int GridPrimalDual::label_height(int row, int column, int c)
{
	int ret = singletonPotential(row, column, c);
	if (row != 0) { // has left
		ret -= y[row-1][column][c].right;
	}
	if (row != rows-1) { // has right
		ret += y[row][column][c].right;
	}
	if (column != 0) { // has up
		ret -= y[row][column-1][c].down;
	}
	if (column != columns-1) { // has down
		ret += y[row][column][c].down;
	}

	return ret;
}

int GridPrimalDual::load(int prow, int pcol, int qrow, int qcol, int a, int b)
{
	if (pcol == qcol) {
		if (prow == qrow + 1) {
			//to right
			return y[prow][pcol][a].right - y[prow][pcol][b].right;
		}
		else if (prow == qrow - 1) {
			//to left
			return y[qrow][qcol][b].right - y[qrow][qcol][a].right;
		} 
		else { throw "not adjacent"; }
	}
	else if (prow == qrow) {
		if (pcol == qcol + 1) {
			//to down
			return y[prow][pcol][a].down - y[prow][pcol][b].down;
		}
		else if (pcol == qcol - 1) {
			// to up
			return y[qrow][qcol][b].down - y[qrow][qcol][a].down;
		}
		else { throw "not adjacent"; }
	}
	throw "not adjacent";
}
