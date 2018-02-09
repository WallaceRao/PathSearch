/*
*          !!!!!!   SPECIFICATION     !!!!!
*
* This is the D* Lite Algorithm implementation for the assignment 1 of course 159740.
*
*							---	Yonghui Rao
*							---	raoyonghui0630@gmail.com
*/
#include <stdio.h>
#include <iostream>
#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>  //sqrt, pow
#include <algorithm>
#include "dstarlite.h"
#include "gridworld.h"

using namespace std;

DStarLite::DStarLite(int rows_, int cols_) {
	rows = rows_;
	cols = cols_;

	//Allocate memory 
	maze.resize(rows);
	for (int i = 0; i < rows; i++) {
		maze[i].resize(cols);
	}
	start = new DstarLiteCell;
	goal = new DstarLiteCell;
	inited = false;
	last = NULL;
}


void DStarLite::resetCellsStatus()  // Mark all cells as not expanded and not accessed before computing shortest path, for statistic purpose
{
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++) {
			maze[i][j].expanded = FALSE;
			maze[i][j].accessed = FALSE;
		}
}

void DStarLite::statCellsStatus(int &numExpanded, int &numAccessed)  // Statistic the number of cells that are expaned / accessed
{
	numExpanded = 0;
	numAccessed = 0;
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++) {
			if (maze[i][j].expanded)
				numExpanded++;
			if (maze[i][j].accessed)
				numAccessed++;
		}
}



void DStarLite::initialise(int startX, int startY, int goalX, int goalY) {;

	km = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			maze[i][j].g = INF;
			maze[i][j].rhs = INF;
			// Init moves and costs
			int moveIndex = 0;
			for (int ii = -1; ii <= 1; ii++)
				for (int jj = -1; jj <= 1; jj++)
				{
					if (ii != 0 || jj != 0)
					{
						int neighbourY = i + ii;
						int neighbourX = j + jj;
						if (neighbourX >= 0 && neighbourY >= 0
							&& neighbourY < rows && neighbourX < cols)
						{
							DstarLiteCell *neighbour = &maze[neighbourY][neighbourX];
							if (neighbour->type == '1' || maze[i][j].type == '1')
								maze[i][j].linkCost[moveIndex] == INF;
							else
								if (HEURISTIC == EUCLIDEAN)
									maze[i][j].linkCost[moveIndex] = sqrt(abs(ii) + abs(jj));
								else
									maze[i][j].linkCost[moveIndex] = 1;
							maze[i][j].move[moveIndex] = &(maze[neighbourY][neighbourX]);
							moveIndex++;
						}
						else
						{
							maze[i][j].linkCost[moveIndex] = INF;
							maze[i][j].move[moveIndex] = NULL;
							moveIndex++;
						}

					}
				}
		}
	}


	//START VERTEX
	start->g = INF;
	start->rhs = INF;
	start->x = startX;
	start->y = startY;

	//GOAL VERTEX
	goal->g = INF;
	goal->rhs = 0.0;
	goal->x = goalX;
	goal->y = goalY;
	//---------------------
	maze[start->y][start->x].g = start->g;
	maze[start->y][start->x].rhs = start->rhs;

	maze[goal->y][goal->x].g = goal->g;
	maze[goal->y][goal->x].rhs = goal->rhs;

	calcKey(&maze[goal->y][goal->x]);
	U.push_back(&maze[goal->y][goal->x]);
	make_heap(U.begin(), U.end(), heapCmp);
	//---------------------
	start = &maze[start->y][start->x];
	goal = &maze[goal->y][goal->x];

	inited = true;
}

double DStarLite::minValue(double g_, double rhs_) {
	if (g_ <= rhs_) {
		return g_;
	}
	else {
		return rhs_;
	}
}

int DStarLite::maxValue(int v1, int v2) {

	if (v1 >= v2) {
		return v1;
	}
	else {
		return v2;
	}
}

double DStarLite::calc_H(DstarLiteCell *cell1, DstarLiteCell *cell2) {
	return estimateDistance(cell1->x, cell1->y, cell2->x, cell2->y);
}



void DStarLite::updateHValues() {
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			maze[i][j].h = calc_H(&maze[i][j], start);
		}
	}
}


// calculate key and save it to cell.
void DStarLite::calcKey(DstarLiteCell *cell) {
	double key1, key2;

	key2 = minValue(cell->g, cell->rhs);
	key1 = key2 + calc_H(start, cell) + km;

	cell->key[0] = key1;
	cell->key[1] = key2;
}




void DStarLite::calcKey(DstarLiteCell *cell, double key[]) {
	double key1, key2;

	key2 = minValue(cell->g, cell->rhs);
	key1 = key2 + calc_H(start, cell) + km;

	key[0] = key1;
	key[1] = key2;
}



// Update one vertex based on its successors
void DStarLite::updateVertex(DstarLiteCell *cell) {
	cell->accessed = true;
	int goalY = goal->y;
	int goalX = goal->x;
	DstarLiteCell *goal = &maze[goalY][goalX];
	DstarLiteCell *pathSucc = NULL;
	if (cell != goal)
	{
		float tempRhs = INF;
		set<int>::iterator iter = cell->succs.begin();
		while (iter != cell->succs.end())
		{

			int succIndex = *iter;
			DstarLiteCell *succ = cell->move[succIndex];


			if (succ && cell->linkCost[succIndex] < INF)
			{
				float gc = succ->g + cell->linkCost[succIndex];
				if (gc < tempRhs)
				{
					tempRhs = gc;
					pathSucc = succ;
				}
			}
			iter++;
		}	
		cell->rhs = tempRhs;
		cell->pathSucc = pathSucc;
	}
	removeFromU(cell);
	if (traversable(cell) && (cell->g != cell->rhs))
	{
		insertToU(cell);
	}
	int currentUSize = U.size();
	if (maxQLength < currentUSize)
	{
		maxQLength = currentUSize;
	}
}

// Remove one cell from the priority queue
void DStarLite::removeFromU(DstarLiteCell *cell)
{
	vector<DstarLiteCell *>::iterator iter = U.begin();
	while (iter != U.end())
	{
		if (cell == *iter)
		{
			U.erase(iter);
			break;
		}
		iter++;
	}
	if (iter != U.end())
		make_heap(U.begin(), U.end(), heapCmp);
}


// Insert one cell to the priority queue

void DStarLite::insertToU(DstarLiteCell *cell)
{
	if (traversable(cell))
	{
		calcKey(cell);
		bool inserted = false;
		vector<DstarLiteCell *>::iterator iter = U.begin();
		while (iter != U.end())
		{
			if (*iter == cell)
			{
				inserted = true;
				break;
			}
			iter++;
		}
		if (!inserted)
		{
			U.push_back(cell);
			push_heap(U.begin(), U.end(), heapCmp);
		}
	}
}


// If the new key is larger than old key, return true, else false.
bool DStarLite::compareKey(double oldk[], double newk[])
{
	if (newk[0] > oldk[0])
		return true;
	else if (newk[0] == oldk[0])
		if(newk[1] > oldk[1])
			return true;
	return false;
}


// Main fuction for computing shortest path based on current knowledge
bool DStarLite::computeShortestPath()
{
	runningSerach = true;
	int startY = start->y;
	int startX = start->x;
	while (1)
	{
		if (U.empty())
		{
			if (start->rhs == start->g && start->rhs < INF)
				return true;
			return false;
		}
		DstarLiteCell *topU = U.front();

		calcKey(start);

		if (compareKey(topU->key, start->key) || (start->rhs != start->g))
		{

			double newKey[2];
			double *oldKey = topU->key;
			calcKey(topU, newKey);

			pop_heap(U.begin(), U.end(), heapCmp);
			U.pop_back();
			if (compareKey(oldKey, newKey))
			{
				topU->expanded = true;
				calcKey(topU);
				insertToU(topU);
				push_heap(U.begin(), U.end(), heapCmp);
			}
			else if (topU->g > topU->rhs) {
				topU->expanded = true;
				topU->g = topU->rhs;
				for (int i = 0; i < 8; i++)
				{
					DstarLiteCell *neighbour = topU->move[i];
					if (neighbour && traversable(neighbour))
					{
						for (int j = 0; j < 8; j++)
						{
							if (neighbour->move[j] == topU)
								neighbour->succs.insert(j);
						}
						updateVertex(neighbour);
					}
				}
			}
			else {
				topU->expanded = true;
				topU->g = INF;
				for (int i = 0; i < 8; i++)
				{
					DstarLiteCell *neighbour = topU->move[i];
					if (neighbour && traversable(neighbour))
					{
						for (int j = 0; j < 8; j++)
						{
							if (neighbour->move[j] == topU)
								neighbour->succs.insert(j);
						}
						updateVertex(neighbour);
					}

				}
				updateVertex(topU);
			}
		}
		else {
			break;
		}
	}
	return true;

}


// If one cell becomes blocked from unblocked, the existing shortest path that before this cell should be recalculated, so re-init them

void DStarLite::disablePathViaCell(DstarLiteCell *cell)
{
	cell->g = INF;
	cell->rhs = INF;
	removeFromU(cell);

	for (int i = 0; i < 8; i++)
	{
		DstarLiteCell *neighbour = cell->move[i];
		if ((neighbour != NULL) && (neighbour->pathSucc == cell))
		{
			removeFromU(neighbour);
			if (neighbour->pathSucc == cell)
				neighbour->pathSucc = NULL;
			disablePathViaCell(neighbour);
		} 
	}
}


// Main fuction for 1st search and replanning

bool DStarLite::dstarliteMain(int startX, int startY, int goalX, int goalY)
{

	initialise(startX, startY, goalX, goalY);
	start = &maze[startY][startX];
	originStart = start;
	goal = &maze[goalY][goalX];

	// At the begining the robot is at the start vertex, the neighbours of the start vertex are detected.
	for (int i = 0; i < 8; i++)
	{
		DstarLiteCell *neighbour = start->move[i];
		if (neighbour && neighbour->type == '8')
			neighbour->type = '0'; // We know it is traverable now.
		// Block its neighbors whose type is 9, update all edges related to this neighbour.
		if (neighbour && (neighbour->type == '9'))
		{
			neighbour->type = '1'; // We know it is blocked now.
			for (int j = 0; j < 8; j++)
			{
				neighbour->linkCost[j] = INF;
				DstarLiteCell *move = neighbour->move[i];
				for (int q = 0; q < 8; q++)
				{
					if (move->move[q] == neighbour)
						move->linkCost[q] = INF;
				}
			}
		}
	}
		
	last = &maze[startY][startX];


	initStatistic();
	resetCellsStatus();
	cout << " begin first search..." << endl;
	long long  t1 = milliseconds_now();
	if (!computeShortestPath())
	{
		cout << "error:  first search failed..." << endl;
		return false;
	}
	long long  t2 = milliseconds_now();
	cout << "1st seatch for shortest path finished, time used: " << (t2 - t1) << " ms" << endl;
	statCellsStatus(numberOfExpandedStates, numberOfVertexAccesses);
	showStatistic();

	// Show the first search result
	updateUI();

	
	DstarLiteCell *ss = &maze[2][4];
	DstarLiteCell *newStart = NULL;
	while (start != goal)
	{
		float min_c_g = INF;

		if (start->succs.size() == 0)
		{
			cout << "error, no succ" << endl;
			return false;
		}
		set<int>::iterator iter = start->succs.begin();
		newStart = NULL;
		while (iter != start->succs.end())
		{

			int succIndex = *iter;
			DstarLiteCell *succ = start->move[succIndex];
			if (succ && succ->type != '1' && succ->type != '9' && start->linkCost[succIndex] < INF)
			{

				if (min_c_g > succ->g + start->linkCost[succIndex])
				{
					min_c_g = succ->g + start->linkCost[succIndex];
					newStart = succ;
				}
				if (newStart == NULL)
					newStart = succ;

			}
			iter++;
		}
		start = newStart;
		cout << "move one step by inputting any key:" << endl;
		getch();
		updateUI();
		// move robot to new start
		// update map...
		// Scan neighbours

		cout << "Move to new start: " << start->y << " " << start->x << endl;
		bool edgeChanged = false;
		for (int i = 0; i < 8; i++)
		{
			DstarLiteCell *neighbour = start->move[i];
			if (neighbour && neighbour->type == '9')
			{
				edgeChanged = true;
			}
			if (neighbour && neighbour->type == '8')
			{
				neighbour->type = '0';
			}
		}
		if (!edgeChanged)
		{
			continue;
		}
		km = km + calc_H(last, start);
		for (int i = 0; i < 8; i++)
		{
			DstarLiteCell *neighbour = start->move[i];
			if (neighbour && neighbour->type == '9')
			{
				DstarLiteCell *newBlockCell = neighbour;
				newBlockCell->type = '1'; // We know it is blocked now, update all related edges
				newBlockCell->g = INF;
				newBlockCell->rhs = INF;
				disablePathViaCell(newBlockCell);
				for (int j = 0; j < 8; j++)
				{
					newBlockCell->linkCost[j] = INF;
					
					DstarLiteCell *move = newBlockCell->move[j]; // Update the edges point to this blocked neighbour
					if (move)
					{
						for (int q = 0; q < 8; q++)
						{
							if (move->move[q] == newBlockCell)
								move->linkCost[q] = INF;
						}
						// Remove the blockCell from the succ set of neighbour 
						set<int>::iterator iter = move->succs.begin();
						while (iter != move->succs.end())
						{
							int succIndex = *iter;
							DstarLiteCell *succ = move->move[succIndex];
							if (succ == newBlockCell)
							{

								move->succs.erase(iter);
								break;
							}
							iter++;

						}
						updateVertex(move);
					}
				}
			}
		}
		cout << "replanning start... " << endl;
		updateHValues();
		initStatistic();
		resetCellsStatus();
		long long  t1 = milliseconds_now();
		if (!computeShortestPath())
		{
			cout << "error, no path found" << endl;
			return false;
		}
		long long  t2 = milliseconds_now();
		cout << "replanning for shortest path finished, time used: " << (t2 - t1) << " ms" << endl;
		statCellsStatus(numberOfExpandedStates, numberOfVertexAccesses);
		showStatistic();
		updateUI();

	}
	
	return true;

}