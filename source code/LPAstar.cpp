/*
*          !!!!!!   SPECIFICATION     !!!!!
*
* This is the LPA* Algorithm implementation for the assignment 1 of course 159740.
*
*							---	Yonghui Rao
*							---	raoyonghui0630@gmail.com
*/


#include <stdio.h>
#include <iostream>
#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>        //sqrt, pow
#include <algorithm>
#include "LPAstar.h"
#include "gridworld.h"


 LpaStar::LpaStar(int rows_, int cols_){
		rows = rows_;
	    cols = cols_;
	 
		 //Allocate memory 
		 maze.resize(rows);
		 for(int i=0; i < rows; i++){
		   maze[i].resize(cols);
		 }
		 start = new LpaStarCell;
		 goal = new LpaStarCell;
		 runningSerach = false;
}



void LpaStar::resetCellsStatus()  // Mark all cells as not expanded and not accessed before computing shortest path, for statistic purpose
{
	for (int i = 0; i < rows; i++)
		for (int j = 0; j < cols; j++) {
			maze[i][j].expanded = FALSE;
			maze[i][j].accessed = FALSE;
		}
}

void LpaStar::statCellsStatus(int &numExpanded, int &numAccessed)  // Statistic the number of cells that are expaned / accessed
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

void LpaStar::initialise(int startX, int startY, int goalX, int goalY){

	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){

		   maze[i][j].g = INF;
			maze[i][j].rhs = INF;
			// Init moves and costs
			int moveIndex = 0;
			for(int ii = -1; ii <=1; ii ++)
				for (int jj = -1; jj <= 1; jj++)
				{
					if (ii != 0 || jj != 0)
					{ 
						int neighbourY = i + ii;
						int neighbourX = j + jj;
						if (neighbourX >= 0 && neighbourY >= 0
							&& neighbourY < rows && neighbourX < cols)
						{
							LpaStarCell *neighbour = &maze[neighbourY][neighbourX];
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
							moveIndex ++;
						}
					
					}
				}
		}
	}
	//START VERTEX

	start->g = INF;
	start->rhs = 0.0;
	start->x = startX;
	start->y = startY;
	
	//GOAL VERTEX

	goal->g = INF;
	goal->rhs = INF;
	goal->x = goalX;
	goal->y = goalY;
	//---------------------
	maze[start->y][start->x].g = start->g;
	maze[start->y][start->x].rhs = start->rhs;
	
	maze[goal->y][goal->x].g = goal->g;
	maze[goal->y][goal->x].rhs = goal->rhs;

	U.push_back(&maze[start->y][start->x]);
	make_heap(U.begin(), U.end(), heapCmp);
	maxQLength = 1;
	//---------------------
	
	inited = true;
}

double LpaStar::minValue(double g_, double rhs_){
	if(g_ <= rhs_){
		return g_;
	} else {
		return rhs_;
	}	
}

int LpaStar::maxValue(int v1, int v2){
	
	if(v1 >= v2){
		return v1;
	} else {
		return v2;
	}	
}

double LpaStar::calc_H(int x, int y){
	return estimateDistance(goal->x, goal->y, x, y);
}

void LpaStar::updateHValues(){
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].h = calc_H(j, i);
		}
	}
	
	start->h = calc_H(start->x, start->y);
	goal->h = calc_H(goal->x, goal->y);
}

void LpaStar::calcKey(int x, int y){
	double key1, key2;
	
	key2 = minValue(maze[y][x].g, maze[y][x].rhs);
	key1 = key2 + maze[y][x].h;
}


void LpaStar::calcKey(LpaStarCell *cell){
	double key1, key2;
	
	key2 = minValue(cell->g, cell->rhs);
	key1 = key2 + cell->h;
	
	cell->key[0] = key1;
	cell->key[1] = key2;
}

void LpaStar::updateAllKeyValues(){	
	cout << "update all key values" << endl;
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   calcKey(&maze[i][j]);
		}
	}
	
	calcKey(start);
	calcKey(goal);
}

// Update one vertex based on its predessors

void LpaStar::updateVertex(LpaStarCell *cell) {
	cell->accessed = true;
	int startY = start->y;
	int startX = start->x;
	LpaStarCell *start = &maze[startY][startX];
	if (cell != start)
	{
		float tempRhs = INF;
		set<int>::iterator iter = cell->preds.begin();
		while (iter != cell->preds.end())
		{
			int predIndex = *iter;
			LpaStarCell *pred = cell->move[predIndex];
			if (pred && cell->linkCost[predIndex] < INF)
			{
				float gc = pred->g + cell->linkCost[predIndex];
				if (gc < tempRhs)
				{
					tempRhs = gc;
					cell->pathPred = pred;
				}
					
			}
			iter++;
		}
		cell->rhs = tempRhs;
	}
	removeFromU(cell);
	if (traversable(cell) && (cell->g != cell->rhs))
	{
		calcKey(cell);
		bool inserted = false;
		vector<LpaStarCell *>::iterator iter = U.begin();
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
	int currentUSize = U.size();
	if (maxQLength < currentUSize)
	{
		maxQLength = currentUSize;
	}
}

// Remove one cell from the priority queue
void LpaStar::removeFromU(LpaStarCell *cell)
{
	vector<LpaStarCell *>::iterator iter = U.begin();
	while (iter!= U.end())
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

bool equal(float f1, float f2)
{
	if (f1 > (f2 - 0.001) && f1 < (f2 + 0.001))
		return true;
	return false;
}

void LpaStar::disablePathViaCell(LpaStarCell *cell)
{
	cell->rhs = INF;
	cell->g = INF;
	removeFromU(cell);
	for (int i = 0; i < 8; i++)
	{
		LpaStarCell *neighbour = cell->move[i];
		if ((neighbour != NULL) && (neighbour->pathPred == cell))
		{
			neighbour->rhs = INF;
			neighbour->g = INF;
			removeFromU(neighbour);
			disablePathViaCell(neighbour);
		}
	}

}


// If the key of cell1 is larger than cell2, return true, else false.
bool LpaStar::compareKey(LpaStarCell *cell1, LpaStarCell *cell2)
{

	if (cell1->key[0] > cell2->key[0])
	{
		return true;
	}
	else if (equal(cell1->key[0], cell2->key[0]))
	{
		if (cell1->key[1] > cell2->key[1])
		{
			return true;
		}
	}
	return false;
}


// Main fuction to compute shortest path
bool LpaStar::computeShortestPath()
{
	runningSerach = true;
	int goalY = goal->y;
	int goalX = goal->x;

	LpaStarCell *goal = &maze[goalY][goalX];
	while (1)
	{

		if (U.empty())
		{
			cout << "U is empty" << endl;
			if(goal->rhs == goal->g && goal->rhs < INF)
				return true;
			return false;
		}
		LpaStarCell *topU = U.front();
		calcKey(goal);
		if (compareKey(goal, topU) || (goal->rhs != goal->g && goal->rhs < INF))
		{
			pop_heap(U.begin(), U.end(),heapCmp);
			U.pop_back();
			topU->expanded = true;
			if (topU->g > topU->rhs)
			{
				topU->g = topU->rhs;
				for (int i = 0; i < 8; i++)
				{
					LpaStarCell *neighbour = topU->move[i];
					if (neighbour && traversable(neighbour))
					{
						for (int j = 0; j < 8; j++)
						{
							if (neighbour->move[j] == topU)
							{
								neighbour->preds.insert(j);
								break;
							}
						}
						updateVertex(neighbour);
					}
				}
			}
			else {
				topU->g = INF;
				
				for (int i = 0; i < 8; i++)
				{
					LpaStarCell *neighbour = topU->move[i];
					if (neighbour && traversable(neighbour))
					{
						for (int j = 0; j < 8; j++)
						{
							if (neighbour->move[j] == topU)
							{
								neighbour->preds.insert(j);
								break;
							}
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

// Mark the maze cell be blocked, return true if success, else false
bool LpaStar::blockCell(int i, int j)
{
	LpaStarCell *blockCell = &maze[i][j];
	if (blockCell->type == '1')
	{
		return false;
	}
	cout << "block cell:" << j << " " << i << endl;
	blockCell->type = '1';
	blockCell->g = INF;
	blockCell->rhs = INF;
	disablePathViaCell(blockCell);
	for (int i = 0; i < 8; i++)
	{
		LpaStarCell *neighbour = blockCell->move[i];
		if (neighbour->type == '1')
			continue;

		if (neighbour != NULL)
		{
			// Remove the blockCell from the pred set of neighbour 
			set<int>::iterator iter = neighbour->preds.begin();
			while (iter!= neighbour->preds.end())
			{
				int predIndex = *iter;
				if (neighbour->move[predIndex] == blockCell)
				{
					neighbour->preds.erase(iter);
					break;
				}
				iter++;

			}
			blockCell->linkCost[i] = INF;

			for (int j = 0; j < 8; j++)
			{
				if (neighbour->move[j] == blockCell)
				{
					neighbour->linkCost[j] = INF;
				}
			}

			updateVertex(neighbour);
		}
	}
	return true;
}



// Not support yet
void LpaStar::unBlockCell(int i, int j)
{
	cout << "unblock has not been enabled yet" << endl;
	return;
}



