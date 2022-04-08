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
	std::cout << rows << " size " << columns << std::endl;
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
	height.reserve(number_of_labels);
	for (int c = 0; c < number_of_labels; c++)
	{
		y.push_back(std::vector< std::vector<FourNeighboors> >());
		y[c].reserve(rows);
		height.push_back(std::vector< std::vector<int> >());
		height[c].reserve(rows);
		for (int row = 0; row < rows; row++) {
			y[c].push_back(std::vector<FourNeighboors>());
			y[c][row].reserve(columns);
			height[c].push_back(std::vector<int>());
			height[c][row].reserve(columns);
			for (int column = 0; column < columns; column++) {
				y[c][row].push_back({ 0,0 });
				height[c][row].push_back(singletonPotential(row, column, c));
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
				//std::cout << row << " " << column << " labelr " << x_p << " " << x_q << std::endl;
				y[x_p][row][column].right = distance(x_p, x_q);
				height[x_p][row][column] += y[x_p][row][column].right;
				height[x_p][row][column+1] -= y[x_p][row][column].right;
			}

			if (row != rows - 1) { // there is a down node
				x_q = x[row + 1][column];
				//std::cout << row << " " << column << " labeld " << x_p << " " << x_q << std::endl;
				y[x_p][row][column].down = distance(x_p, x_q); //from 0 to wpq dist(x_p,x_q)
				height[x_p][row][column] += y[x_p][row][column].down;
				height[x_p][row+1][column] -= y[x_p][row][column].down;
			}

			//std::cout << "hey" << std::endl;

			//int count_ys = 0;

			//int count_bad_ys = 0;
			//for (int row2 = 0; row2 < rows; row2++) {
			//	for (int column2 = 0; column2 < columns; column2++)
			//	{

			//		if (row2 > 0) { // has up
			//			count_ys -= y[x[row2][column2]][row2 - 1][column2].down;

			//			count_bad_ys -= y[x[row2 - 1][column2]][row2 - 1][column2].down;
			//		}
			//		if (column2 > 0) { // has left
			//			count_ys -= y[x[row2][column2]][row2][column2 - 1].right;

			//			count_bad_ys -= y[x[row2][column2 - 1]][row2][column2 - 1].right;
			//		}


			//		if (row2 < rows - 1) {  // has down

			//			count_ys += y[x[row2][column2]][row2][column2].down;

			//			count_bad_ys += y[x[row2 + 1][column2]][row2][column2].down;
			//		}
			//		if (column2 < columns - 1) { // has right

			//			count_ys += y[x[row2][column2]][row2][column2].right;

			//			count_bad_ys += y[x[row2][column2 + 1]][row][column].right;
			//		}



			//	}
			//}
			//int afp = compute_APF_cost();
			//if ( row != rows - 1)
			//	std::cout << "checking " << row << " " << column << " " << count_ys << " " << count_bad_ys << " " << afp << " " << height[x_p][row][column] << " " << height[x_p][row + 1][column] << std::endl;





		}
	}


	//height.reserve(number_of_labels);
	//for (int c = 0; c < number_of_labels; c++)
	//{
	//	height.push_back(std::vector< std::vector<int> >());
	//	height[c].reserve(rows);
	//	for (int row = 0; row < rows; row++) {
	//		height[c].push_back(std::vector<int>());
	//		height[c][row].reserve(columns);
	//		for (int column = 0; column < columns; column++) {
	//			height[c][row].push_back(singletonPotential(row, column, c));
	//			if (row > 0) { // has up
	//				height[c][row][column] -= y[c][row - 1][column].down;
	//			}
	//			if (column > 0) { // has left
	//				height[c][row][column] -= y[c][row][column - 1].right;
	//			}


	//			if (row < rows - 1) {  // has down

	//				height[c][row][column] += y[c][row][column].down;
	//			}
	//			if (column < column - 1) { // has right

	//				height[c][row][column] += y[c][row][column].right;
	//			}



	//		}
	//	}

	//}
	/*std::cout << compute_APF_cost() << std::endl;



	check();*/

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
	loop_count = 0;
	bool cont = true;
	while (cont) {
		s_linked_per_outer.push_back(0);
		std::cout << "loop n " << loop_count << std::endl;
		for (int row = 0; row < rows; row++)
		{
			for (int column = 0; column < columns; column++)
			{
				x_old[row][column] = x[row][column];
			}
		}
		cont = false;

		for (int c = 0; c < number_of_labels; c++) {
			std::cout << "innerloop c " << c << std::endl;
			//check();
			preEditDuals(c);
			updateDualsPrimals(c);
			postEditDuals(c);

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
					height[c][row][column] -= y[c][row][column].right;
					height[c][row][column + 1] += y[c][row][column].right;
					y[c][row][column].right = distance(c, x_q) - y_qp_xq;
					height[c][row][column] += y[c][row][column].right;
					height[c][row][column + 1] -= y[c][row][column].right;
				}
			}

			if (row != rows - 1) { // there is a down node
				x_q = x[row + 1][column];
				int y_pq_c = y[c][row][column].down;
				int y_qp_xq = -y[x_q][row][column].down;
				int y_pq_xp = y[x_p][row][column].down;
				int y_qp_c = -y[c][row][column].down;


				if ((y_pq_c + y_qp_xq > distance(c, x_q)) || (y_pq_xp + y_qp_c > distance(x_p, c))) {
					height[c][row][column] -= y[c][row][column].down;
					height[c][row+1][column] += y[c][row][column].down;
					y[c][row][column].down = distance(c, x_q) - y_qp_xq;
					height[c][row][column] += y[c][row][column].down;
					height[c][row+1][column] -= y[c][row][column].down;
					
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
			
			diff_height = height[x[row][column]][row][column] - height[c][row][column];
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
	s_linked_per_outer[loop_count] += s_link;
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
					height[c][row][column] += pflow;
					height[c][row + 1][column] -= pflow;
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
					height[c][row][column] += pflow;
					height[c][row][column + 1] -= pflow;
					
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
	if (!g->has_run()) return; // no max_flow, no change possible
	int x_p, x_q;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) {
			x_p = x[row][column];
			//if (g->has_nonsat_path_to_source(pos2nodeIndex[row][column])) {
				if (column != columns - 1) { // there is a right node
					x_q = x[row][column + 1];

					int y_pq_xp = y[x_p][row][column].right;
					int y_qp_xq = -y[x_q][row][column].right;


					if (y_pq_xp + y_qp_xq > distance(x_p, x_q)) {
						
						height[c][row][column] -= y[c][row][column].right;
						height[c][row][column+1] += y[c][row][column].right;
						// either x_p=c or x_q=c
						if (x_p == c) {
							y[c][row][column].right = distance(x_p, x_q) + y[x_q][row][column].right;
						}
						else if (x_q == c) {
							y[c][row][column].right = -distance(x_p, x_q) + y[x_p][row][column].right;
						}
						height[c][row][column] += y[c][row][column].right;
						height[c][row][column + 1] -= y[c][row][column].right;

					}
				}

				if (row != rows - 1) { // there is a down node
					x_q = x[row + 1][column];
					int y_pq_xp = y[x_p][row][column].down;
					int y_qp_xq = -y[x_q][row][column].down;


					if (y_pq_xp + y_qp_xq > distance(x_p, x_q)) {


						height[c][row][column] -= y[c][row][column].down;
						height[c][row + 1][column] += y[c][row][column].down;

						// either x_p=c or x_q=c
						if (x_p == c) {
							y[c][row][column].down = distance(x_p, x_q) + y[x_q][row][column].down;
						}
						else if (x_q == c) {
							y[c][row][column].down = -distance(x_p, x_q) + y[x_p][row][column].down;
						} 
						height[c][row][column] += y[c][row][column].down;
						height[c][row + 1][column] -= y[c][row][column].down;

					}
				}
			//}
		}
	}
}

int GridPrimalDual::compute_primal_cost()
{
	int cost = 0;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++)
		{
			int tmp_cost = singletonPotential(row, column, x[row][column]);
			if (row  < rows -1 ) { //has left
				tmp_cost += distance(x[row][column], x[row +1][column]);
			}
			if (column < columns - 1) { //has up
				tmp_cost += distance(x[row][column], x[row][column + 1]);
			}

			cost += tmp_cost;
		}
	}
	return cost;
}

int GridPrimalDual::compute_dual_cost()
{
	int cost = 0;
	int lowest_height;
	int lowest_height_label;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++) 
		{
			lowest_height = INT_MAX;
			for (int c = 0; c < number_of_labels; c++) {
				if (height[c][row][column] < lowest_height) { lowest_height = height[c][row][column]; lowest_height_label = c; }
			}
			//if (row == 100 && column == 100) std::cout << "dual " << lowest_height_label << " " << lowest_height << std::endl;
			cost += lowest_height;
		}
	}
	return cost;
}

int GridPrimalDual::compute_APF_cost() {
	int cost = 0;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++)
		{
			
			cost += height[x[row][column]][row][column];
			//if (row == 100 && column == 100) std::cout << "APF " << x[row][column] << " " << height[x[row][column]][row][column] << std::endl;
		}
	}
	return cost;
}

int GridPrimalDual::compute_height_depth()
{
	int depth = 0;
	int lowest_height;
	int lowest_height_label;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++)
		{
			lowest_height = INT_MAX;
			for (int c = 0; c < number_of_labels; c++) {
				if (height[c][row][column] < lowest_height) { lowest_height = height[c][row][column]; lowest_height_label = c; }
			}
			depth += lowest_height - height[x[row][column]][row][column];
		}
	}
	return depth;
}

void GridPrimalDual::check()
{
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++)
		{
			if (row < rows-1) {
				if (load(row, column, row + 1, column, x[row][column], x[row + 1][column]) != distance(x[row][column], x[row + 1][column])) {
					std::cout << row << " " << column << " " << load(row, column, row + 1, column, x[row][column], x[row + 1][column]) << " " << distance(x[row][column], x[row + 1][column]) << std::endl;
					exit(-1010);
				}
				if (load(row, column, row + 1, column, x[row][column], x[row + 1][column]) != y[x[row][column]][row][column].down - y[x[row + 1][column]][row][column].down) {
					std::cout << row << " " << column << " " << load(row, column, row + 1, column, x[row][column], x[row + 1][column]) << " " << y[x[row][column]][row][column].down << " " << y[x[row + 1][column]][row][column].down << std::endl;
					exit(-211);
				}
			}
			if (column < columns - 1) {
				if (load(row, column, row, column + 1, x[row][column], x[row][column + 1]) != distance(x[row][column], x[row][column + 1])) {
					std::cout << row << " " << column << " " << load(row, column, row, column+1, x[row][column], x[row][column+1]) << " " << distance(x[row][column], x[row][column+1]) << std::endl;
					exit(-312);
				}
				if (load(row, column, row, column + 1, x[row][column], x[row][column + 1]) != y[x[row][column]][row][column].right - y[x[row][column + 1]][row][column].right) {
					std::cout << row << " " << column << " " << load(row, column, row, column + 1, x[row][column], x[row][column + 1]) << " " << y[x[row][column]][row][column].right << " " << y[x[row][column + 1]][row][column].right << std::endl;
					exit(-413);
				}
			}
		}
	}
	int count_singleton = 0;
	int count_dist = 0;
	int count_ys = 0;

	//int count_bad_ys = 0;
	for (int row = 0; row < rows; row++) {
		for (int column = 0; column < columns; column++)
		{

			if (row > 0 ) { // has up
				count_ys -= y[x[row][column]][row-1][column].down;

				//count_bad_ys -= y[x[row - 1][column]][row - 1][column].down;
			}
			if (column > 0 ) { // has left
				count_ys -= y[x[row][column]][row][column-1].right;

				//count_bad_ys -= y[x[row][column - 1]][row][column - 1].right;
			}


			if (row < rows - 1) {  // has down
				count_dist += distance(x[row][column], x[row + 1][column]);

				count_ys += y[x[row][column]][row][column].down;

				//count_bad_ys += y[x[row + 1][column]][row][column].down;
			}
			if (column < columns - 1) { // has right
				count_dist += distance(x[row][column], x[row][column + 1]);

				count_ys += y[x[row][column]][row][column].right;

				//count_bad_ys += y[x[row][column + 1]][row][column].right;
			}

			count_singleton += singletonPotential(row, column, x[row][column]);


		}
	}
	int afp = compute_APF_cost();
	if (count_dist != count_ys || count_singleton+ count_dist != afp) {
		std::cout << count_dist << " " << count_ys << " " << std::endl;

		std::cout << count_singleton + count_dist << " " << count_singleton + count_ys << " " << afp  << std::endl;// << " " << count_singleton + count_bad_ys << std::endl;
		exit(-13);
	}

	//std::cout << "good " << count_singleton + count_dist << " " << count_singleton + count_ys << " " << afp << std::endl;// << " " << count_singleton + count_bad_ys << std::endl;




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

