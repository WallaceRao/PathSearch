///////////////////////////////////////////////////////////////////////////////////////////
//
//	 	      Program Name: globalVariables 
//
//	 		        History:  date of revision
//                         13/Aug/2017
//                         28/July/2015  
//                         03/Aug/2014  
//
//      Start-up code by:    n.h.reyes@massey.ac.nz
//
///////////////////////////////////////////////////////////////////////////////////////////


/*
*          !!!!!!   SPECIFICATION     !!!!!
*
* Modified for the assignment 1 of course 159740 on 18/Sep/2017.
*
*							---	Yonghui Rao
*							---	raoyonghui0630@gmail.com
*/


#ifndef __GLOBALVARIABLES_H__
#define __GLOBALVARIABLES_H__


#include <vector>
#include <set>
#include "windows.h"
#include <iostream>
/*******************************************************************************************************************/
//------------------------------------------
//Flags - enable or disable features

using namespace std;

#define EIGHT_CONNECTED_GRIDWORLD
//#define FOUR_CONNECTED_GRIDWORLD

//#define SHOW_DEBUG_INFO false
//#define SHOW_DEBUG_INFO true

#define MANHATTAN	1
#define EUCLIDEAN	2


#define LPASTAR 1
#define DSTARLITE 2

//------------------------------------------


//-------------------------------------------------------------------------------
#ifdef FOUR_CONNECTED_GRIDWORLD
 
	//4-connected gridworld
	#define DIRECTIONS 4
	const struct {
	  int x;
	  int y;
	} neighbours[4]={{0, -1}, {-1, 0}, {1, 0}, {0, 1}};

	/////////////////////////////////////////////////////

#endif



const double SQRT_2 =  1.4142135623731;

//-------------------------------------------------------------------------------
#ifdef EIGHT_CONNECTED_GRIDWORLD

	//8-connected gridworld
	#define DIRECTIONS 8
	
	//movement sequence, used in the journal
	const struct {
	  int x;
	  int y;
	} neighbours[8]={ {-1,-1}, {0, -1}, {1, -1}, 
					{-1, 0}, {1, 0}, 
					{-1, 1}, {0, 1}, {1, 1} };
			
	//clockwise, starting at 3 o'clock			
	//~ const struct {
	  //~ int x;
	  //~ int y;
	//~ } succ[8]={ {1,0}, {1, 1}, {0,1}, {-1, 1}, {-1, 0}, {-1,-1}, {0, -1}, {1, -1} };
		
#endif
//-------------------------------------------------------------------------------

//------------------------------------------
	
 extern int numberOfExpandedStates;
 extern int numberOfVertexAccesses;
 extern int maxQLength;
 extern int qLengthAfterSearch;  // Not uesd for the table
 extern int pathLength;



 extern bool showExpandedCells;

 long long milliseconds_now();



 inline void initStatistic()
 {
	
	 numberOfExpandedStates = 0;
	 numberOfVertexAccesses = 0;
	 maxQLength = 0;
	 pathLength = 0;
 }

 inline void showStatistic()
 {
	 cout << " Number of max queue length is " << maxQLength << endl;
	 cout << " Number of state expansions is " << numberOfExpandedStates << endl;
	 cout << " Number of accessed vertexs is " << numberOfVertexAccesses << endl;
 }

	




extern bool MAP_INITIALISED;
extern bool PRECALCULATED_GRIDWORLD_READY;

//~ extern bool USE_EUCLIDEAN_DISTANCE_HEURISTIC;
extern unsigned int HEURISTIC;
extern unsigned int ALGORITHM;

//Robot soccer dimensions	
extern int GRIDWORLD_ROWS; 
extern int GRIDWORLD_COLS;

//////////////////////////////////////////////////////////////////////////////
//REINFORCEMENT LEARNING

//extern int MAX_ACTIONS;

//end_REINFORCEMENT LEARNING
//////////////////////////////////////////////////////////////////////////////
using namespace std;

enum cellType{TRAVERSABLE=0, BLOCKED=1, UNKNOWN=9};
enum vertexStatus{UNEXPLORED=0, EXPANDED=1, ACCESSED=2};

 struct CellPosition
{
	int row;
	int col;
};

 struct Coordinates
{
	int x, y;
};


typedef struct {
  int y;
  int x;
} loc_t;



struct vertex
{
	double rhs;
    double g;
	// int c;
	double h;
	double f;
	double key[2];
	vertex* move[DIRECTIONS]; 
    double linkCost[DIRECTIONS];	

	//--------------------------------------------------------------------------------- 
	//TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
    char type; 
	//---------------------------------------------------------------------------------
	int row;
	int col;
	char status; 
	bool expanded;
	 
	int x1,y1,x2,y2;
	Coordinates centre; //centre x, centre y
}; 

extern int MAX_MOVES;

extern vector<vector<vertex> > map;
extern vertex startVertex;
extern vertex goalVertex;



typedef struct vector<CellPosition> PathType;

/*******************************************************************************************************************/
extern int fieldX1, fieldY1, fieldX2, fieldY2; //playing field boundaries
extern float WORLD_MAXX;
extern float WORLD_MAXY;
// colour constants
extern int BACKGROUND_COLOUR;
extern int LINE_COLOUR;

extern int cellWidth;
extern int cellHeight;

extern int NAPOLEON;

/********************************************************************************************************************/
//8-connected gridworld
#define DIRECTIONS 8 //clockwise sequence of moves (8-connected gridworld)

#define INF  1000000


struct LpaStarCell
{
    LpaStarCell* move[DIRECTIONS];
	set<int> preds;  // Used for LPA*, save all its neighbours which are its predessors
	set<int> succs;     // Used for LPA*, save all its neighbours which are its successors
	LpaStarCell *pathPred;   // Used for LPA*, if robot is at cell c, c->pathPred means the cell that the robot should move to in next step
	LpaStarCell *pathSucc;   // Used for D* Lite, if robot is at cell c, c->pathSucc means the previous of the robot
	
	double linkCost[DIRECTIONS];

    int x, y;

	double g;
    double rhs;
	double h;
    double key[2];
	
	bool accessed;
	bool expanded;
	 //---------------------
	 //TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
	 char type;
	 //----------------------
};
typedef LpaStarCell DstarLiteCell;  // LPA* and D* Lite use the same cell struct.

extern bool SHOW_MAP_DETAILS;

inline bool traversable(LpaStarCell *cell)
{
	if(ALGORITHM == LPASTAR)
		if (cell->type == '1'  || cell->type == '9')
			return false;
	if(ALGORITHM == DSTARLITE)
		if (cell->type == '1')
			return false;
	return true;
}


// Used to maintain the min heap.

inline bool heapCmp(LpaStarCell *cell1, LpaStarCell *cell2)
{
	if ((cell1->h + cell1->rhs) > (cell2->h + cell2->rhs))
	{
		return true;
	}
	return false;
}

// Get latest start cell, for drawing the robot.
DstarLiteCell *getDstarLiteLastStart();

// Update UI after every robot's step.
void updateUI();

#endif
