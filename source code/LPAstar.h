/*
*          !!!!!!   SPECIFICATION     !!!!!
*
* This is the LPA* Algorithm implementation for the assignment 1 of course 159740.
*
*							---	Yonghui Rao
*							---	raoyonghui0630@gmail.com
*/

#ifndef __LPASTAR_H__
#define __LPASTAR_H__

#include <vector> 
#include "globalVariables.h"

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

class LpaStar{

public:
    LpaStar(int rows, int cols); //constructor

    void initialise(int startX, int startY, int goalX, int goalY);
	 double minValue(double g_, double rhs_);
    //double maxValue(double v1, double v2);
    int maxValue(int v1, int v2);
	 void calcKey(int x, int y);
    void calcKey(LpaStarCell *cell);
    //void calc_H(int x, int y);
    double calc_H(int x, int y);
    void updateHValues();
    void updateAllKeyValues();
	void updateVertex(LpaStarCell *cell);
	bool computeShortestPath();
	void removeFromU(LpaStarCell *cell);
	bool compareKey(LpaStarCell *cell1, LpaStarCell *cell2);
	bool blockCell(int, int);
	void unBlockCell(int, int);
	void resetCellsStatus();  // Mark all cells as not expanded and not accessed before computing shortest path, for statistic purpose
	void statCellsStatus(int &numExpanded, int &numAccessed);   // Statistic the number of cells that are expaned / accessed.
	void disablePathViaCell(LpaStarCell *cell);
    //void copyMazeToDisplayMap(GridWorld gWorld);
    friend void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa);
    friend void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa);
	bool inited;
	bool runningSerach;

private:
	
    vector<vector<LpaStarCell> > maze;   
    LpaStarCell l;
    vector<LpaStarCell* > U; //Priority Queue
    LpaStarCell* start;
    LpaStarCell* goal;
	
    int rows;
    int cols;
	

};

#endif
