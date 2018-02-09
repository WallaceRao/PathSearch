///////////////////////////////////////////////////////////////////////////////////////////
//
//
//  
//                        
//
//	 	      Program Name: Incremental Search 
//	 	       Description: start-up codes for simulating LPA* and D*Lite
//                        - implements a gridworld class that loads a gridworld from file, and is
//                          modifiable through a user-interface 
//
//        Run Parameters: 
//
//    Keys for Operation: 
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


#include <windows.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <deque>
#include <set>
#include <vector>
#include <algorithm>

//-------------------------
#include "globalVariables.h"
#include "transform.h"
#include "graphics.h"
#include "AstarSearch.h"
#include "LPAstar.h"
#include "dstarlite.h"
#include "gridworld.h"

bool showExpandedCells = false;
// colour constants
int BACKGROUND_COLOUR;
int LINE_COLOUR;

int robotWidth;
int GRIDWORLD_ROWS; //duplicated in GridWorld
int GRIDWORLD_COLS; //duplicated in GridWorld


//----------------------------
unsigned int HEURISTIC = EUCLIDEAN; // By default EUCLIDEAN distance is adopted.
unsigned int ALGORITHM = LPASTAR; // By default lpastar algorithm is adopted. 
//~ bool USE_EUCLIDEAN_DISTANCE_HEURISTIC;


int numberOfExpandedStates;
int numberOfVertexAccesses;
int maxQLength;
int qLengthAfterSearch;  // Not uesd for the table
int pathLength;




///////////////////////////////////////////////////////////////////////////////
LpaStar* lpa_star;
DStarLite *dstar_lite;

GridWorld grid_world;

bool SHOW_MAP_DETAILS;
BOOL path_found = false;
///////////////////////////////////////////////////////////////////////////////


long long milliseconds_now()
{
	static LARGE_INTEGER s_frequency;
	static BOOL s_use_qpc = QueryPerformanceFrequency(&s_frequency);
	if (s_use_qpc) {
		LARGE_INTEGER now;
		QueryPerformanceCounter(&now);
		return (1000LL * now.QuadPart) / s_frequency.QuadPart;
	}
	else {
		return GetTickCount();
	}
}



// Get the latest start cell from D* Lite
DstarLiteCell *getDstarLiteLastStart()
{
	if (dstar_lite && dstar_lite->inited && dstar_lite->start)
	{
		DstarLiteCell *newstart = dstar_lite->start;
		return newstart;
	}
	return NULL;
}



//--------------------------------------------------------------
//copy maze (from LPA*) to map (of GridWorld)
void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa){
	for(int i=0; i < gWorld.getGridWorldRows(); i++){
	   for(int j=0; j < gWorld.getGridWorldCols(); j++){
			gWorld.map[i][j].type = lpa->maze[i][j].type;
		    gWorld.map[i][j].h = lpa->maze[i][j].h;

			gWorld.map[i][j].g = lpa->maze[i][j].g;
			gWorld.map[i][j].rhs = lpa->maze[i][j].rhs;
			gWorld.map[i][j].row = lpa->maze[i][j].y;
			gWorld.map[i][j].col = lpa->maze[i][j].x;
			gWorld.map[i][j].expanded = lpa->maze[i][j].expanded;
			for(int k=0; k < 2; k++){
			  gWorld.map[i][j].key[k] = lpa->maze[i][j].key[k];			  
			}
			for (int q = 0; q < 8; q++)
			{
				gWorld.map[i][j].linkCost[q] = lpa->maze[i][j].linkCost[q];
			}
			
		}
	}

}

void copyMazeToDisplayMap(GridWorld &gWorld, DStarLite* dstarlite) {


	for (int i = 0; i < gWorld.getGridWorldRows(); i++) {
		for (int j = 0; j < gWorld.getGridWorldCols(); j++) {
			gWorld.map[i][j].type = dstarlite->maze[i][j].type;
			gWorld.map[i][j].h = dstarlite->maze[i][j].h;
			gWorld.map[i][j].g = dstarlite->maze[i][j].g;
			gWorld.map[i][j].rhs = dstarlite->maze[i][j].rhs;
			gWorld.map[i][j].row = dstarlite->maze[i][j].y;
			gWorld.map[i][j].col = dstarlite->maze[i][j].x;
			gWorld.map[i][j].expanded = dstarlite->maze[i][j].expanded;
			for (int k = 0; k < 2; k++) {
				gWorld.map[i][j].key[k] = dstarlite->maze[i][j].key[k];
			}
			for (int q = 0; q < 8; q++)
			{
				gWorld.map[i][j].linkCost[q] = dstarlite->maze[i][j].linkCost[q];
			}

		}
	}

}




//--------------------------------------------------------------
//copy map (of GridWorld)to maze (of LPA*)
void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa){
	for(int i=0; i < gWorld.getGridWorldRows(); i++){
	   for(int j=0; j < gWorld.getGridWorldCols(); j++){
		   if (gWorld.map[i][j].type == '8')
			   gWorld.map[i][j].type = '0';
			if(gWorld.map[i][j].type == '9')
			   gWorld.map[i][j].type = '1';
			lpa->maze[i][j].type = gWorld.map[i][j].type;
			lpa->maze[i][j].x = gWorld.map[i][j].col;
			lpa->maze[i][j].y = gWorld.map[i][j].row;
			
		   //lpa->maze[i][j].g = gWorld.map[i][j].g;
			//lpa->maze[i][j].rhs = gWorld.map[i][j].rhs;
		}
	}
	
	vertex startV = gWorld.getStartVertex();
	vertex goalV = gWorld.getGoalVertex();
	
	//lpa->start->g = gWorld.map[startV.row][startV.col].g ;
	//lpa->start->rhs = gWorld.map[startV.row][startV.col].rhs ;
	lpa->start->x = gWorld.map[startV.row][startV.col].col;
	lpa->start->y = gWorld.map[startV.row][startV.col].row;
	
	//lpa->goal->g = gWorld.map[goalV.row][goalV.col].g;
	//lpa->goal->rhs = gWorld.map[goalV.row][goalV.col].rhs;
	lpa->goal->x = gWorld.map[goalV.row][goalV.col].col;
	lpa->goal->y = gWorld.map[goalV.row][goalV.col].row;
	
}


//--------------------------------------------------------------
//copy map (of GridWorld)to maze (of DStarLite)
void copyDisplayMapToMaze(GridWorld &gWorld, DStarLite* dstarlite) {
	for (int i = 0; i < gWorld.getGridWorldRows(); i++) {
		for (int j = 0; j < gWorld.getGridWorldCols(); j++) {
			dstarlite->maze[i][j].type = gWorld.map[i][j].type;
			dstarlite->maze[i][j].x = gWorld.map[i][j].col;
			dstarlite->maze[i][j].y = gWorld.map[i][j].row;

			//lpa->maze[i][j].g = gWorld.map[i][j].g;
			//lpa->maze[i][j].rhs = gWorld.map[i][j].rhs;
		}
	}

	vertex startV = gWorld.getStartVertex();
	vertex goalV = gWorld.getGoalVertex();

	//lpa->start->g = gWorld.map[startV.row][startV.col].g ;
	//lpa->start->rhs = gWorld.map[startV.row][startV.col].rhs ;
	dstarlite->start->x = gWorld.map[startV.row][startV.col].col;
	dstarlite->start->y = gWorld.map[startV.row][startV.col].row;

	//lpa->goal->g = gWorld.map[goalV.row][goalV.col].g;
	//lpa->goal->rhs = gWorld.map[goalV.row][goalV.col].rhs;
	dstarlite->goal->x = gWorld.map[goalV.row][goalV.col].col;
	dstarlite->goal->y = gWorld.map[goalV.row][goalV.col].row;

}

///////////////////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES


///////////////////////////////////////////////////////////////////////////////
// IMPLEMENTATION
///////////////////////////////////////////////////////////////////////////////




void drawInformationPanel(int x, int y, char* info){
   ///////////////////////////////////////////////////////////////////////////////////////////
	settextstyle(SMALL_FONT, HORIZ_DIR, 4);
	settextjustify(LEFT_TEXT,CENTER_TEXT);
	setcolor(YELLOW);
	outtextxy(x ,y, info);
	///////////////////////////////////////////////////////////////////////////////////////////
}

int getKey(){
	
	 if(GetAsyncKeyState(VK_UP) < 0) { //UP ARROW
       return 200;
    } 
	 
	 if(GetAsyncKeyState(VK_DOWN) < 0) { //DOWN ARROW
       return 201;
    }
	
    if(GetAsyncKeyState(VK_F4) < 0) { 
       SHOW_MAP_DETAILS=false;
		 return 104;
    } 
  
    if(GetAsyncKeyState(VK_F5) < 0) {
        SHOW_MAP_DETAILS=true;
		  return 105;
    }
	 
	 if(GetAsyncKeyState(VK_F6) < 0) {
        //highlight expanded cells
		  return 106;
    }
	 if(GetAsyncKeyState(VK_F7) < 0) {
        // cancel highlighting expanded cells
		  return 107;
    }
	 if(GetAsyncKeyState(VK_F8) < 0) {
        //execute D*Lite
		  return 108;
    }
	 
	 //copy display map to algorithm's maze
	 if(GetAsyncKeyState(VK_F9) < 0) {
		  return 109;
    }
	 
	 //copy algorithm's maze to display map
	 if(GetAsyncKeyState(VK_F10) < 0) {
		  return 110;
    }
	 	 
	 if(GetAsyncKeyState(0x53) < 0) { //S-key (start cell)
		  return 6;
    }
	 
	 if(GetAsyncKeyState(0x58) < 0) { //X-key (goal cell)
		  return 7;
    }
	 
	 if(GetAsyncKeyState(0x42) < 0) { //B-key (block cell)
		  return 1;
    }
	 
	 if(GetAsyncKeyState(0x47) < 0) {  //G-key
		  return 9;
    }
	 
	 if(GetAsyncKeyState(0x48) < 0) {  //H-key
		  return 10;
    }
	 
	 if(GetAsyncKeyState(0x4B) < 0) {  //K-key
		  return 11;
    }
	 
	 if(GetAsyncKeyState(0x55) < 0) { //U-key (Unblock cell)
		  return 12;
    }
	 
	 if(GetAsyncKeyState(0x50) < 0) { //P-key (position of cells)
		  return 14;
    }
	 
	 if(GetAsyncKeyState(0x43) < 0) { //C-key (connections of cells)
		  return 15;
    }
	 
	 if(GetAsyncKeyState(0x4D) < 0) { //M-key (entire map connections)
		  return 16;
    }
	 
	 if(GetAsyncKeyState(0x52) < 0) { //R-key (REINFORCEMENT LEARNING - reward values)
		  return 17;
    }
	 
	 if(GetAsyncKeyState(0x51) < 0) { //Q-key (REINFORCEMENT LEARNING - maxQ-values)
		  return 18;
    }
	 
	if(GetAsyncKeyState(VK_SPACE) < 0) { //SPACE BAR
		  return 1000;
    } 

    if(GetAsyncKeyState(VK_RETURN) < 0) { //ENTER KEY
		  return 1001;
    }

    
 }
 static BOOL page = false;
void runSimulation(char *fileName){
	WorldBoundaryType worldBoundary; //duplicated in GridWorld
    DevBoundaryType deviceBoundary; //duplicated in GridWorld
	bool ANIMATE_MOUSE_FLAG=false;
	bool validCellSelected=false;
	
	int mX, mY;
	float worldX, worldY;
	worldX=0.0f;
	worldY=0.0f;
	
	int action=-1;
	//-----------------------
	CellPosition p;
	int rowSelected, colSelected;
	//-----------------------
    rowSelected=-1;
	colSelected=-1;
	
	int mouseRadius=1;
		
	srand(time(NULL));  // Seed the random number generator
			
	//Initialise the world boundaries
    grid_world.initSystemOfCoordinates();
	grid_world.loadMapAndDisplay(fileName);
	grid_world.initialiseMapConnections();
	
	//----------------------------------------------------------------
	//LPA*
	lpa_star = new LpaStar(grid_world.getGridWorldRows(), grid_world.getGridWorldCols());

	dstar_lite = new DStarLite(grid_world.getGridWorldRows(), grid_world.getGridWorldCols());

	vertex start = grid_world.getStartVertex();
	vertex goal = grid_world.getGoalVertex();
	
	cout << "(start.col = " << start.col << ", start.row = " << start.row << ")" << endl;
	cout << "(goal.col = " << goal.col << ", goal.row = " << goal.row << ")" << endl;
	
	//lpa_star->initialise(start.col, start.row, goal.col, goal.row);
	
	//lpa_star->copyMazeToDisplayMap(grid_world);
	//copyMazeToDisplayMap(grid_world, lpa_star);
	//copyDisplayMapToMaze(grid_world, lpa_star);
	//----------------------------------------------------------------
		
	worldBoundary = grid_world.getWorldBoundary();
	deviceBoundary = grid_world.getDeviceBoundary();
	GRIDWORLD_ROWS = grid_world.getGridWorldRows();
	GRIDWORLD_COLS = grid_world.getGridWorldCols();
	
	//setvisualpage(page);
	
	// keep running the program until the ESC key is pressed   
	while((GetAsyncKeyState(VK_ESCAPE)) == 0 ) {
			 setactivepage(page);
			 cleardevice();
	
		     action = getKey(); 
		
		     if(SHOW_MAP_DETAILS) 
				 grid_world.displayMapWithDetails();
			 else
			     grid_world.displayMap();
			 if (path_found && ALGORITHM == LPASTAR)
			 {
				 grid_world.displayPath();
			 }
			 switch(action){
			 	case 1000:

			 	          break;

                case 1001:  //ENTER KEY

                         //calc shortest path

                         break;  

				case 1: //Block selected cell
				 		
                        if( (rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS) && (colSelected > 1) && (colSelected < GRIDWORLD_COLS)){
							grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '1');
							
							if (ALGORITHM == LPASTAR && lpa_star->runningSerach)
							{
								if (lpa_star->blockCell(rowSelected - 1, colSelected - 1))
								{
									path_found = false;
									initStatistic();
									lpa_star->resetCellsStatus();
									long long  t1 = milliseconds_now();
									cout << "replanning start..." << endl;
									path_found = lpa_star->computeShortestPath();
									if (path_found)
									{
										long long  t2 = milliseconds_now();
										cout << "replanning finished, time used: " << (t2 - t1) << " ms" << endl;
										lpa_star->statCellsStatus(numberOfExpandedStates, numberOfVertexAccesses);
										showStatistic();
									}
									else {
										cout << "replanning failed: no path found." << endl;
									}
									copyMazeToDisplayMap(grid_world, lpa_star);
									if (path_found)
									{
										grid_world.displayPath();
									}
								}
							}
							rowSelected=-1;
							colSelected=-1;
						}
						action = -1;
						break;
				
				case 105: 
					   grid_world.displayMapWithKeyDetails();
					break;
				
				case 106: 
				    	cout << "Highlighting expanded cells has been turned on" << endl;
					   showExpandedCells = true;
						break;
				
				case 107: 
					   cout << "Highlighting expanded cells has been turned on" << endl;
					   showExpandedCells = false;; 
						break;
				
				case 108: 
					  
					   //~ algorithmSelection = DSTAR_ALGORITHM;
						break;
				
				case 15:
					 
					    if( rowSelected != -1 && colSelected != -1){
						   grid_world.displayVertexConnections(colSelected-1, rowSelected-1);
						   //cout << "display connections" << endl;
						   rowSelected=-1;
						   colSelected=-1;
					    } else {
							cout << "invalid new START vertex, please select a new START vertex first." << endl;
							break;
						}
						//--------------------------------------------
					    action = -1;
					    break;
						
				case 16:
					 
					    if(grid_world.isGridMapInitialised()){
							grid_world.displayMapConnections();
						   //cout << "display connections" << endl;
						   //~ rowSelected=-1;
						   //~ colSelected=-1;
					    } else {
							cout << "map has not been initialised yet." << endl;
							break;
						}
						//--------------------------------------------
					    action = -1;
					    break;		
				
				case 6: //set cell as new START vertex 
				   {
					    //--------------------------------------------
				        // retrieve current START vertex
				        vertex s = grid_world.getStartVertex();
				        if( (rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS) && (colSelected > 1) && (colSelected < GRIDWORLD_COLS)) {
					        if( (s.row != -1) && (s.col != -1) ){
							
								//set current START VERTEX to an ordinary TRAVERSABLE CELL
								grid_world.setMapTypeValue(s.row, s.col, '0'); 
								grid_world.initialiseMapConnections(); 
								cout << "new START vertex:" << s.row << " "<< s.col << endl;
								//ok, proceed
							} else {
								cout << "invalid START vertex" << endl;
								break;
							}
					    }
				      //--------------------------------------------
						//set selected cell as the NEW START VERTEX
					   if( (rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS) && (colSelected > 1) && (colSelected < GRIDWORLD_COLS)) {
						   grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '6');
						    s.row = rowSelected-1;
							s.col = colSelected-1;
							grid_world.setStartVertex(s);
							
						   rowSelected=-1;
						   colSelected=-1;
					   } else {
							cout << "invalid new START vertex, please select a new START vertex first." << endl;
							break;
						}
						//--------------------------------------------
					   action = -1;
						break;
					}
				
				case 7: //set cell as new GOAL vertex 
					   {


						   //--------------------------------------------
					      // retrieve current GOAL vertex
					      vertex s = 	grid_world.getGoalVertex();
					      if( (rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS) && (colSelected > 1) && (colSelected < GRIDWORLD_COLS)) {
						       if( (s.row > 1) && (s.row < GRIDWORLD_ROWS)  && (s.col > 1) && (s.col < GRIDWORLD_COLS) ){
									//set current GOAL VERTEX to an ordinary TRAVERSABLE CELL
									grid_world.setMapTypeValue(s.row, s.col, '0'); 
									
									//ok, proceed
								} else {
									cout << "invalid GOAL vertex" << endl;
									action = -1;
									break;
								}
						   }
					      //--------------------------------------------
							//set selected cell as the NEW GOAL VERTEX
						   if( (rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS) && (colSelected > 1) && (colSelected < GRIDWORLD_COLS)) {
							   grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '7');
							   s.row = rowSelected-1;
							   s.col = colSelected-1;
							   grid_world.setGoalVertex(s);
							   grid_world.initialiseMapConnections(); 
								
							   rowSelected=-1;
							   colSelected=-1;
						   } else {
								cout << "invalid new GOAL vertex, please select a new GOAL vertex first." << endl;
								action = -1;
								break;
							}
							//--------------------------------------------
						   action = -1;
							break;
						}
							
                case 109:
					  start = grid_world.getStartVertex();
					  goal = grid_world.getGoalVertex();
					  if (ALGORITHM == LPASTAR)
					  {
						  copyDisplayMapToMaze(grid_world, lpa_star);
						  lpa_star->initialise(start.col, start.row, goal.col, goal.row);
					  }
					  else {
						  copyDisplayMapToMaze(grid_world, dstar_lite);
						  dstar_lite->initialise(start.col, start.row, goal.col, goal.row);
					  }
				      cout << "copied display map to algorithm's maze" << endl;
				      action = -1;
				      break;
				
				case 110:	
					  if (ALGORITHM == LPASTAR)
					  {
						  if (!path_found)
						  {
							  lpa_star->updateHValues();
							  initStatistic();
							  lpa_star->resetCellsStatus();
							  long long  t1 = milliseconds_now();
							  cout << "1st seatch for shortest path start..." << endl;
							  path_found = lpa_star->computeShortestPath();
							  if (path_found)
							  {
								  long long  t2 = milliseconds_now();
								  cout << "1st seatch for shortest path finished, time used: " << (t2 - t1) << " ms" << endl;
								  lpa_star->statCellsStatus(numberOfExpandedStates, numberOfVertexAccesses);
								  showStatistic();
							  }
							  else {
								  cout << "1st seatch failed: no path found." << endl;
							  }
							  copyMazeToDisplayMap(grid_world, lpa_star);
							  if (path_found)
							  {
								  grid_world.displayPath();
							  }
						  }
					  }
					  else {
						  if (!path_found)
						  {
							  dstar_lite->updateHValues();
							  path_found = dstar_lite->dstarliteMain(start.col, start.row, goal.col, goal.row);
							  copyMazeToDisplayMap(grid_world, dstar_lite);
						  }
					
					  }
					 
				       action = -1;
				       break;
				
				case 9: //display g-values only
					   grid_world.displayMapWithSelectedDetails(true, false, false, false);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
				       action = -1;
					   break;
                case 10: //display h-values only
					    grid_world.displayMapWithSelectedDetails(false, false, true, false);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
				 		action = -1;
				        break;
				case 11: //display key-values only
					    lpa_star->updateAllKeyValues();
				        copyMazeToDisplayMap(grid_world, lpa_star);
					    grid_world.displayMapWithSelectedDetails(false, false, false, true);  //(bool display_g, bool display_rhs, bool display_h, bool display_key) 
						action = -1;
				        break;
				
				case 12: //make cell Traversable // unblock
			 
					 if( (rowSelected > 1) && (rowSelected < GRIDWORLD_ROWS) && (colSelected > 1) && (colSelected < GRIDWORLD_COLS)){

						 grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '0');
						 if (ALGORITHM == LPASTAR)
						 {
							 lpa_star->unBlockCell(rowSelected - 1, colSelected - 1);
							 if (lpa_star->runningSerach)
							 {
								 cout << "replanning start..." << endl;
								 path_found = lpa_star->computeShortestPath();
								 cout << "replanning finished, statistics: " << endl;

								 copyMazeToDisplayMap(grid_world, lpa_star);
								 if (path_found)
								 {
									 grid_world.displayPath();
								 }
							 }
						 }
						
						 rowSelected=-1;
						 colSelected=-1;
					 }
					 action = -1;
					 break; 
					 
				case 14: 
					   grid_world.displayMapWithPositionDetails();
						action = -1;
				      break;	 
					 
				 //~ default: //Display grid without details
					   //~ grid_world.displayMap();
						  
				  
				 
		    };
		


		
	   //----------------------------------------------------------------------------------------------------------------	  
		// Mouse handling
		//
			 if(mousedown()){
						 				
				ANIMATE_MOUSE_FLAG=true;
				 			 
				mX = mousecurrentx();
				mY = mousecurrenty();
				 
				//if the goal selected is within the playing field boundaries
				if(mX >= grid_world.getFieldX1() && mX <= grid_world.getGridMaxX() && mY >= grid_world.getFieldY1() && mY <= grid_world.getGridMaxY()){
					
					    circle(mX, mY, 3);
					    validCellSelected = true;
  	            
				} else {
					validCellSelected = false;
				}
			 } //end of mousedown()
			 //------------------------------------------------------------------------------------------------------------------
			 /////////////////////////////////////////////////////////////////////////////
			 						 
			 if(ANIMATE_MOUSE_FLAG){					
				  //draw Cross-hair to mark Goal	    
				   setcolor(RED);
				   circle(mX, mY, 20);
				   line(mX,mY-20,mX,mY+20);
				   line(mX-20,mY,mX+20,mY);
				   //end of draw Cross-hair 
			 
				   // special effect to display concentric circles locating the target
					setcolor(YELLOW);
					
					if(mouseRadius < 40) {
						mouseRadius += 1;
					}
					circle(mX, mY, mouseRadius);
					//Sleep(50);
									
					if(mouseRadius >= 40) {
						ANIMATE_MOUSE_FLAG=false;
						mouseRadius=0;
					}
					//end of special effect
			  }

			 
			 /////////////////////////////////////////////////////////////////////////////
			  char info[80]; 
			  float wX, wY;
			  
			  wX = xWorld(worldBoundary,deviceBoundary,mX);
			  wY = yWorld(worldBoundary,deviceBoundary,mY);
			  sprintf(info,"x: %d, y: %d",mX, mY); 
			  drawInformationPanel(grid_world.getFieldX2(), grid_world.getFieldY1() + textheight("H")*2, info);
			  
			 
			  sprintf(info,"wX: %3.0f, wY: %3.0f",wX, wY); 
			  drawInformationPanel(grid_world.getFieldX2(),grid_world.getFieldY1() + textheight("H")*5, info);
			 ///////////////////////////////////////////////////////////////////////////// 
			 
			  //~ CellPosition p;
			  //~ int rowSelected, colSelected;
			  
			  if(validCellSelected) {
				  p=grid_world.getCellPosition_markCell(mX, mY);

                 

					  rowSelected = p.row;
					  colSelected = p.col;
					  
					  sprintf(info,"row: %d, col: %d",rowSelected, colSelected); 
				      drawInformationPanel(grid_world.getFieldX2(),grid_world.getFieldY1() + textheight("H")*6, info);
			      
				  
			  }
			  setvisualpage(page);
			  page = !page;  //switch to another page
	}
}



///////////////////////////////////////////////////////////////////////////////////////
//
// EXAMPLE:  main grid_Dstar_journal.txt MANHATTAN

int main(int argc, char *argv[]) {	

	char gridFileName[80];
	cout << "Command syntax:  main <gridworld> {MANHATTAN, EUCLIDEAN} {lpastar, dstarlite}" << endl;
	if (argc == 1) {
		
		cout << "Error: grid file name is not specified, exit..." << endl;
		return 0;
	}
	else if (argc == 2) {
		cout << "Warn: Algorithm and Heuristic function are not specified, use default value." << endl;
		strcpy(gridFileName, argv[1]);
	}
	else if (argc == 3) {
		cout << "Warn: Algorithm is not specified, use default value." << endl;
		string heuristic(argv[2]);
		std::transform(heuristic.begin(), heuristic.end(), heuristic.begin(), ::tolower);
		strcpy(gridFileName, argv[1]);
        
		 //heuristic function selection   		
		if((heuristic.compare("euclidean")==0) || (heuristic.compare("e")==0)){
			HEURISTIC = EUCLIDEAN;			
			cout << "Heuristics = EUCLIDEAN" << endl;
		}
		if((heuristic.compare("manhattan")==0) || (heuristic.compare("m")==0)){		
			HEURISTIC = MANHATTAN;
			cout << "Heuristics = MANHATTAN" << endl;
		}	
	}
	else if (argc == 4) {
		string heuristic(argv[2]);
		string algorithm(argv[3]);
		std::transform(heuristic.begin(), heuristic.end(), heuristic.begin(), ::tolower);
		std::transform(algorithm.begin(), algorithm.end(), algorithm.begin(), ::tolower);
		strcpy(gridFileName, argv[1]);

		//heuristic function selection   		
		if ((heuristic.compare("euclidean") == 0) || (heuristic.compare("e") == 0)) {
			HEURISTIC = EUCLIDEAN;
			cout << "Heuristics = EUCLIDEAN" << endl;
		}
		if ((heuristic.compare("manhattan") == 0) || (heuristic.compare("m") == 0)) {
			HEURISTIC = MANHATTAN;
			cout << "Heuristics = MANHATTAN" << endl;
		}
		if ((algorithm.compare("lpastar") == 0) || (algorithm.compare("l") == 0)) {
			ALGORITHM = LPASTAR;
			cout << "ALGORITHM = LPASTAR" << endl;
		}
		if ((algorithm.compare("dstarlite") == 0) || (algorithm.compare("d") == 0)) {
			ALGORITHM = DSTARLITE;
			cout << "ALGORITHM = dstarlite" << endl;
		}

	} else {
		cout << "\n===================================================================================" << endl;
		//cout << endl << endl;
		cout << "                 << Incremental Search v.1.0   by n.h.reyes@massey.ac.nz>>                                " << endl;
		cout << "===================================================================================" << endl;
		cout << "Syntax error: Too more parameters:  gridworld heuristic algorithm" << endl;
		cout << "\nPlease follow the following syntax:  main <gridworld> {MANHATTAN, EUCLIDEAN} {lpastar, dstarlite}" << endl;
		cout << "\ne.g.  main grid_Dstar_journal.txt MANHATTAN lpastar" << endl;
		cout << "\n===================================================================================" << endl;
	}

	if (ALGORITHM == LPASTAR)
	{
		cout << "LPASTAR algorithm is adopted." << endl;
	}
	else {
		cout << "DSTARLITE algorithm is adopted." << endl;
	}

	if (HEURISTIC == EUCLIDEAN)
	{
		cout << "EUCLIDEAN distance is adopted." << endl;
	}
	else {
		cout << "MANHATTAN distance is adopted." << endl;
	}
   int graphDriver = 0,graphMode = 0;
 	
	//initgraph(&graphDriver, &graphMode, "", 1440, 900); // Start Window
 	//initgraph(&graphDriver, &graphMode, "", 1280, 1024); // Start Window
	
   initgraph(&graphDriver, &graphMode, "", 1360, 768); // Start Window - LAPTOP SCREEN
	//initgraph(&graphDriver, &graphMode, "", 1920, 1080); // Start Window - Full-HD
	
   BACKGROUND_COLOUR = WHITE;
   LINE_COLOUR = GREEN;
	
   GRIDWORLD_ROWS = 0; //7; //6; //duplicated in GridWorld
   GRIDWORLD_COLS = 0; //15;//13; //duplicated in GridWorld
   SHOW_MAP_DETAILS=false;

   try{
		runSimulation(gridFileName);
   }

   //Type of the exceptions thrown by the standard definitions of operator new and operator new[] 
   //when they fail to allocate the requested storage space
   catch(std::bad_alloc & ba){ 
		std::cerr << "out of memory caught: " << ba.what() << endl;
   }

   catch(exception & e){
		cout << "Standard exception: " << e.what() << endl;
   }
	
   catch(...){
    	cout << "Unknown exception caught!\n";
   }
	
	
	cout << "----<< The End.>>----" << endl;
	
	return 0;
} 


//Update UI to reflect robot positon and path once the robot moves one step in D* lite
void updateUI()
{
	if (ALGORITHM != DSTARLITE)
		return;
	setactivepage(page);
	cleardevice();
	copyMazeToDisplayMap(grid_world, dstar_lite);
	if (SHOW_MAP_DETAILS)
		grid_world.displayMapWithDetails();
	else
		grid_world.displayMap();

	grid_world.displayPath();
	
	setvisualpage(page);
	page = !page;  //switch to another page
}