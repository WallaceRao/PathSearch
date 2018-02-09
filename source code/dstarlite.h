
/*
*          !!!!!!   SPECIFICATION     !!!!!
*
* This is the D* Lite Algorithm implementation for the assignment 1 of course 159740.
*
*							---	Yonghui Rao
*							---	raoyonghui0630@gmail.com
*/
#ifndef __DSTARLITE_H__
#define __DSTARLITE_H__

#include <vector> 
#include "globalVariables.h"


class GridWorld; //forward declare class GridWorld to be able to create the friend functions later


struct  cell_key
{
	DstarLiteCell *cell;
	float key[2];

};


inline bool dstarLiteHeapCmp(DstarLiteCell *cell1, DstarLiteCell *cell2)
{

	if ((cell1->h + cell1->rhs) > (cell2->h + cell2->rhs))
	{
		return true;
	}
	return false;
}



class DStarLite {

public:
	DStarLite(int rows, int cols); //constructor

	void initialise(int startX, int startY, int goalX, int goalY);
	double minValue(double g_, double rhs_);
	//double maxValue(double v1, double v2);
	int maxValue(int v1, int v2);
	void calcKey(DstarLiteCell *cell);
	void calcKey(DstarLiteCell *cell, double key[]);
	//void calc_H(int x, int y);
	double calc_H(DstarLiteCell *cell1, DstarLiteCell *cell2);

	void updateVertex(DstarLiteCell *cell);
	bool computeShortestPath();
	void removeFromU(DstarLiteCell *cell);
	void insertToU(DstarLiteCell *cell);

	bool compareKey(double oldk[], double newk[]);

	void disablePathViaCell(DstarLiteCell *cell);

	bool dstarliteMain(int startX, int startY, int goalX, int goalY);
	void updateHValues();

	void resetCellsStatus();  // Mark all cells as not expanded and not accessed before computing shortest path, for statistic purpose
	void statCellsStatus(int &numExpanded, int &numAccessed);   // Statistic the number of cells that are expaned / accessed.

	//void copyMazeToDisplayMap(GridWorld gWorld);
	friend void copyMazeToDisplayMap(GridWorld &gWorld, DStarLite* lpa);
	friend void copyDisplayMapToMaze(GridWorld &gWorld, DStarLite* lpa);

	friend DstarLiteCell *getDstarLiteLastStart();


	bool inited;
	bool runningSerach;

private:

	vector<vector<DstarLiteCell> > maze;
	vector<DstarLiteCell *> U; //Priority Queue
	DstarLiteCell* originStart;
	DstarLiteCell* start;
	DstarLiteCell* last;
	DstarLiteCell* goal;

	float km ;
	int rows;
	int cols;
};

#endif
