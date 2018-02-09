///////////////////////////////////////////////////////////////////////////////////////////
//
//	 	      Program Name: gridworld 
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


#include "gridworld.h"
#include "math.h"
using namespace std;

void GridWorld::displayHeader(){
		 int x,y;
		 x = getmaxx() /2;
		 y = textheight("H")*1;
		 settextstyle(SMALL_FONT, HORIZ_DIR, 5);
		 settextjustify(CENTER_TEXT,TOP_TEXT);
		 setcolor(YELLOW);				
		 setbkcolor(BLACK);
		 if(ALGORITHM == LPASTAR)
			 outtextxy(x ,y, "INCREMENTAL OPTIMAL SEARCH (8-CONNECTED GRIDWORLD) -- LPA*");
		 else
			 outtextxy(x, y, "INCREMENTAL OPTIMAL SEARCH (8-CONNECTED GRIDWORLD) -- D* Lite");
		 y = y + textheight("_");
	    settextjustify(LEFT_TEXT,TOP_TEXT);    
	    outtextxy(fieldX2-textwidth("start-up codes by n.h.reyes@massey.ac.nz"),fieldY2,"start-up codes by n.h.reyes@massey.ac.nz");
	    settextjustify(CENTER_TEXT,TOP_TEXT);    
	    setcolor(GREEN);
		 outtextxy(x ,y, "F4: hide details, F5: show details, F6: highlight expanded cells, F7: turn off highlighting, F9: copy display map to Algorithm data structure(maze), F10: run Search");
	    y = y + textheight("_");
	    setcolor(WHITE);
   	    outtextxy(x ,y, "B: block cell, U: unblock cell, H: h-values, K: key-values, S: new START, X: new GOAL, P: cell positions, C: local connections, M: all connections ");
	    
	    y = y + textheight("_");
	    setcolor(WHITE);
	
	    char info[256];
		 char heuristicStr[80];
	    //-------------------------------------------
		 if (HEURISTIC == EUCLIDEAN){
			strcpy(heuristicStr,"EUCLIDEAN");			
		 }else if (HEURISTIC == MANHATTAN){
			strcpy(heuristicStr,"MANHATTAN");			
		 }	
	    //-------------------------------------------
	    sprintf(info,"fileName = %s, heuristic = %s", fileName, heuristicStr);
   	 //outtextxy(x ,y, "fileName: ");
	    outtextxy(x ,y, info);
}


float estimateDistance(int x1, int y1, int x2, int y2)
{
	int diffY = abs(y1 - y2);
	int diffX = abs(x1 - x2);
	if (HEURISTIC == EUCLIDEAN) {
		float dis = 0;
		if (diffY > diffX) {
			 dis = sqrt(2) * diffX + (diffY - diffX);
		}
		else {
			dis = sqrt(2) * diffY + (diffX - diffY);
		}
		return dis;
	}
	else if (HEURISTIC == MANHATTAN) {
		return diffY > diffX ? diffY : diffX;
	}
}


void GridWorld::displayRLHeader(long maxEpoch){
		 int x,y;
		 x = getmaxx() /2;
		 y = textheight("H")*1;
		 settextstyle(SMALL_FONT, HORIZ_DIR, 7);
		 settextjustify(CENTER_TEXT,TOP_TEXT);
		 setcolor(YELLOW);
				
		 setbkcolor(BLACK);
       		
		 outtextxy(x ,y, "REINFORCEMENT LEARNING DEMO(Q-LEARNING)   by n.h.reyes@massey.ac.nz");
		 y = y + textheight("_");
	    setcolor(GREEN);
		 outtextxy(x ,y, "F4: hide details, F5: show details, F8: Search, F9: Trace shortest path");
	    y = y + textheight("_");
	    setcolor(WHITE);
   	 outtextxy(x ,y, "B: block cell, U: unblock cell, R: reward-values, Q: maxQ-values, S: new START, X: new GOAL, P: cell positions");
	    
	    y = y + textheight("_");
	    setcolor(WHITE);
	
	    char info[150];
	
	    sprintf(info,"fileName = %s", fileName);
   	 //outtextxy(x ,y, "fileName: ");
	    outtextxy(x ,y, info);
		 
		 y = y + textheight("_");
	    setcolor(GREEN);
	    sprintf(info,"MAX_EPOCH = %ld", maxEpoch);
		 outtextxy(x ,y, info);
}

void GridWorld::loadMapAndDisplay(const char* fn) //,int totalRows, int totalColumns)
{
	int cellX, cellY;
	ifstream i_file;
	
	
	
	i_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
	
	try{
		
		
		i_file.open(fn, std::ifstream::in);
		cout << "file opened." << endl;
		strcpy(fileName, fn);
		
		//---------------------------------------
		displayHeader();
		//---------------------------------------
		
		i_file >> GRIDWORLD_ROWS;
		i_file >> GRIDWORLD_COLS;
		//-------------------------------
		xInc = abs(fieldX2-fieldX1)/GRIDWORLD_COLS;
	    cellWidth = xInc;
	    yInc = abs(fieldY2-fieldY1)/GRIDWORLD_ROWS;
        cellHeight = yInc;	
		//-------------------------------
		
		map.resize(GRIDWORLD_ROWS); 
			
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				map[j].resize(GRIDWORLD_COLS);
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					map[j][i].g = 0.0;
					map[j][i].rhs = 0.0;
					map[j][i].f = 0.0;
					map[j][i].row = j;
					map[j][i].col = i;
					map[j][i].key[0] = 0.0;
					map[j][i].key[1] = 0.0;
					
					for(int m=0; m < DIRECTIONS; m++){
					   map[j][i].move[m] = NULL;  	
						map[j][i].linkCost[m] = 1.0; 
					}
					
					//TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
					
					i_file >> map[j][i].type;
					
					if(map[j][i].type == '0') //traversable cell
					{						
						map[j][i].centre = markCell_col_row(i,j, LIGHTGRAY, WHITE);
						//drawCell_RowCol(LIGHTGRAY, j,i, totalRows, totalColumns, NULL);
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						map[j][i].centre = markCell_col_row(i,j, BLACK, WHITE);
						//drawCell_RowCol(BLACK, j,i, totalRows, totalColumns, NULL);
					}
					if(map[j][i].type == '9') //unknown
					{
						map[j][i].centre = markCell_col_row(i,j, 47, WHITE);
						//drawCell_RowCol(DARKGRAY, j,i, totalRows, totalColumns, NULL);
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						map[j][i].centre = markCell_col_row(i,j, GREEN, WHITE);
						//drawCell_RowCol(GREEN, j,i, totalRows, totalColumns, "S");
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						map[j][i].centre = markCell_col_row(i,j, BLUE, WHITE);
						//drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, "G");
					}
					
					if(map[j][i].type == '4') //'SUB-GOAL'
					{
						map[j][i].centre = markCell_col_row(i,j, 47, WHITE);
						//drawCell_RowCol(BLUE, j,i, totalRows, totalColumns, "G");
					}
					
				}
			}
		
		i_file.close();
		cout << "file closed." << endl;
		
		MAP_INITIALISED = true;
	}
	catch (std::ifstream::failure e) {
    std::cerr << "Exception opening/reading/closing file\n";
  }
 
  cout << "\nGridworld loaded from file: " << fn << "." << endl;
  cout << "GRIDWORLD_ROWS = " << GRIDWORLD_ROWS << endl;
  cout << "GRIDWORLD_COLS = " << GRIDWORLD_COLS << endl;
  //getch();
}


void GridWorld::displayPath()
{
	//return;
	if (ALGORITHM == LPASTAR)
	{
		displaylpastarPath();
	}
	else {
		displayDstarLitePath();
	}
}

//find path based on g-values and direct link cost
void GridWorld::displayDstarLitePath()
{
	static double oldLength = 0;
	int currentLength = 0;
	DstarLiteCell *lastStart = getDstarLiteLastStart();

	if (!lastStart)
		return;
   int cellX, cellY;
   vertex* originVertex;
   vertex* neighbour;
   vertex* min_neighbour;
   int neighbourY, neighbourX;
   double linkCost,g;
   double min_linkCost,min_g;
	
   double totalPath=0.0;

   g=0.0;
   linkCost=0.0;
   min_linkCost=0.0;
   min_g=0.0;
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
		
		double min_g_plus_c = INF;
		originVertex = &map[lastStart->y][lastStart->x];
		//cout << "startVertex at location[x=" << originVertex->col << ",y=" << originVertex->row << "]" << endl;
		while(1){
			currentLength++;
		      min_g_plus_c = INF; //new - napoleon
			for(int m=0; m < DIRECTIONS; m++){
				neighbour = originVertex->move[m];
				if(neighbour != NULL && neighbour->type != '1' ){
 
                    linkCost = originVertex->linkCost[m];
					g=(originVertex->move[m])->g;

								
     				if(min_g_plus_c > sum(g,linkCost)){
						min_g_plus_c = sum(g,linkCost);
                        min_g=g;
                        min_linkCost = linkCost;  
						min_neighbour = neighbour;
					}
							
				}  							
			}						
			totalPath = totalPath + min_linkCost;
				
			setcolor(RED);
			setlinestyle(SOLID_LINE, 1, 1);
			line(min_neighbour->centre.x, min_neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
			
			originVertex = min_neighbour;
		//	cout << "now origin is " << originVertex->row << originVertex->col << endl;
			if(originVertex == &map[goalVertex.row][goalVertex.col]) break;
						
		}
		
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception displaying Map\n";
   }

   if (totalPath != oldLength)
   {
	   cout << "current path length is " << totalPath << endl;
	   oldLength = totalPath;
   }
}


//find path based on g-values and direct link cost
void GridWorld::displaylpastarPath()
{
	static double oldPathLength = 0;
	int cellX, cellY;
	vertex* originVertex;
	vertex* neighbour;
	vertex* min_neighbour;
	int neighbourY, neighbourX;
	double linkCost, g;
	double min_linkCost, min_g;

	double totalPath = 0.0;

	g = 0.0;
	linkCost = 0.0;
	min_linkCost = 0.0;
	min_g = 0.0;

	try {
		//---------------------------------------
		displayHeader();
		//---------------------------------------		
		double min_g_plus_c = INF;
		originVertex = &map[goalVertex.row][goalVertex.col];
		
		//cout << "startVertex at location[x=" << originVertex->col << ",y=" << originVertex->row << "]" << endl;
		while (1) {
			//cout << "originVertex is " << originVertex->row << " " << originVertex->col << ", g: " << originVertex->g << endl;
			min_g_plus_c = INF; //new - napoleon
			for (int m = 0; m < DIRECTIONS; m++) {
				neighbour = originVertex->move[m];
				if (neighbour != NULL && neighbour->type != '1' && neighbour->type != '9') {

					if (originVertex->row == 4 && originVertex->col == 31)
					{
						//cout << "(4,31)neighbour " << "(x,y) " << neighbour ->row << " " << neighbour ->col<< ", g and cost: " << neighbour->g << " " << neighbour->linkCost[m] << endl;
					}
					linkCost = originVertex->linkCost[m];
					g = (originVertex->move[m])->g;
					if (min_g_plus_c > sum(g, linkCost)) {
						min_g_plus_c = sum(g, linkCost);
						min_g = g;
						min_linkCost = linkCost;
						min_neighbour = neighbour;
					}

				}
			}
			if (originVertex->row == 4 && originVertex->col == 31)
			{
				//cout << "min neigbour is " << min_neighbour->row << " " << min_neighbour->col << "g: " << min_neighbour->g << endl;
			}
			
		//	cout << "min cost:" << min_linkCost << endl;
			totalPath = totalPath + min_linkCost;
			//cout << "totalPath:" << totalPath << endl;
			setcolor(GREEN);
			setlinestyle(SOLID_LINE, 2, 2);
			line(min_neighbour->centre.x, min_neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);


			originVertex = min_neighbour;

			if (originVertex == &map[startVertex.row][startVertex.col]) break;

		}

	}
	catch (std::ifstream::failure e) {
		std::cerr << "Exception displaying Map\n";
	}

	if (totalPath != oldPathLength)
	{
		oldPathLength = totalPath;
		cout << "current path length is " << totalPath  << endl;
	}
		
}




void GridWorld::initialiseMapConnections() 
{
	int cellX, cellY;
	vertex* originVertex;
	vertex* neighbour;
	int neighbourY, neighbourX;
	
	neighbourY=-1;
	neighbourX=-1;
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					originVertex = &map[j][i];
					for(int m=0; m < DIRECTIONS; m++){
						
						//originVertex->linkCost[m] = INF;
						
						neighbourY = originVertex->row + neighbours[m].y;
						neighbourX = originVertex->col + neighbours[m].x;
						//for debugging only
						//cout << "neighbour["<< m << "]: (neighbourX = " << neighbourX << ", neighbourY = " << neighbourY << ")" << endl;
						if( (neighbourX >= 0) && (neighbourX < GRIDWORLD_COLS) && (neighbourY >= 0) && (neighbourY < GRIDWORLD_ROWS)){
						   neighbour = &map[neighbourY][neighbourX];
						   
							if(neighbour->type != '1'){ //if the neighbour is not BLOCKED, then it is reacheable
							   //map[j][i].move[m] = neighbour;
								originVertex->move[m] = neighbour;
								originVertex->linkCost[m] = 1.0;
							} else if(neighbour->type == '1'){
								
								originVertex->move[m] = neighbour; //THIS IS ONLY A TEST
								originVertex->linkCost[m] = INF;
							}
							
						}
					   
					}
					
					
					if(map[j][i].type == '0') //traversable cell
					{						
						//markCell_col_row(i,j, LIGHTGRAY, WHITE);
						
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						//markCell_col_row(i,j, BLACK, WHITE);
						
					}
					if(map[j][i].type == '9') //unknown
					{
						//markCell_col_row(i,j, DARKGRAY, WHITE);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						//markCell_col_row(i,j, GREEN, WHITE);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						//markCell_col_row(i,j, BLUE, WHITE);
						
					}
				}
			}
		
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception dispaying Map\n";
   }
  
}

//inputs: col, row of vertex 
//output: display vertex connections
void GridWorld::displayVertexConnections(int i, int j) 
{
	int cellX, cellY;
	vertex* originVertex;
	vertex* neighbour;
	int neighbourY, neighbourX;
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
		
		cout << "vertex(x = " << i << ", y= " << j << endl;
			//for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			//{
				
				//for(int i =0;i < GRIDWORLD_COLS; i++) //col
				//{
		
		         //If the cell is not a BLOCKED CELL
		         if(map[j][i].type != '1'){
		
						originVertex = &map[j][i];
						for(int m=0; m < DIRECTIONS; m++){
							neighbour = map[j][i].move[m];
							if(neighbour != NULL && neighbour->type != '1'){
								cout << "cost[" << m << "] = " << originVertex->linkCost[m] << endl; 
								//setcolor(RED);
								setcolor(YELLOW);
								setlinestyle(WIDE_DOT_FILL, 2, 2);
							   line(neighbour->centre.x, neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
							}  else if(neighbour->type == '1'){
								setcolor(RED);
								setlinestyle(WIDE_DOT_FILL, 2, 2);
							   line(neighbour->centre.x, neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
								cout << "cost[" << m << "] = " << originVertex->linkCost[m] << endl; //<< "blocked cell" << endl; 
								
							}
							
						}
				   } 
					
			
			setlinestyle(SOLID_LINE, 1, 1);
		
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception dispaying Map\n";
   }
  
 
  
}


//output: display entire map connections
//~ void GridWorld::displayRLShortestPath(bool display_reward, bool display_maxQ) 
//~ {
	//~ if(shortestPath.size() <= 0){
		//~ cout << "calculate shortest path first." << endl;
		
	//~ }
	
	//~ vertex* from;
	//~ vertex* to;
		
	//~ loc_t wayPoint;
	
	//~ try{
		//~ //---------------------------------------
		 //~ //displayHeader();
		//~ //---------------------------------------		
		   //~ for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			//~ {
				
				//~ for(int i =0;i < GRIDWORLD_COLS; i++) //col
				//~ {
					
					//~ if(map[j][i].type == '0') //traversable cell
					//~ {						
						//~ markCell_col_row_RL_details(i,j, LIGHTGRAY, WHITE, display_reward, display_maxQ);
					//~ }
					
					//~ if(map[j][i].type == '1') //'B' - blocked cell
					//~ {
						//~ markCell_col_row_RL_details(i,j, BLACK, WHITE, display_reward, display_maxQ);
						
					//~ }
					//~ if(map[j][i].type == '9') //unknown
					//~ {
						//~ markCell_col_row_RL_details(i,j, DARKGRAY, WHITE, display_reward, display_maxQ);
						
					//~ }
					//~ if(map[j][i].type == '6') //'S' - start vertex
					//~ {
						//~ startVertex.row = j;
						//~ startVertex.col = i;
						//~ markCell_col_row_RL_details(i,j, GREEN, WHITE, display_reward, display_maxQ);
						
					//~ }
					//~ if(map[j][i].type == '7') //'G' - goal vertex
					//~ {
						//~ goalVertex.row = j;
						//~ goalVertex.col = i;
						//~ markCell_col_row_RL_details(i,j, BLUE, WHITE, display_reward, display_maxQ);
						
					//~ }
					//~ if(map[j][i].type == '4') //sub GOAL
					//~ {
						//~ goalVertex.row = j;
						//~ goalVertex.col = i;
						//~ markCell_col_row_RL_details(i,j, CYAN, WHITE, display_reward, display_maxQ);
						
					//~ }
					
				//~ }
			//~ }
		
		
		//~ //---------------------------------------
		//~ wayPoint = shortestPath[0];
		//~ from = &map[wayPoint.y][wayPoint.x];
		//~ for(int i=0; i < shortestPath.size(); i++){
			//~ wayPoint = shortestPath[i];
			
			//~ to = &map[wayPoint.y][wayPoint.x];
			
			//~ setcolor(RED);
			//~ setlinestyle(WIDE_DOT_FILL, 2, 2);
			//~ line(from->centre.x, from->centre.y, to->centre.x, to->centre.y);	
			
			//~ from = to;

		//~ }
					
	//~ }
   //~ catch (std::ifstream::failure e) {
    //~ std::cerr << "Exception dispaying Map\n";
   //~ }
  
 
  
//~ }


//output: display entire map connections
void GridWorld::displayMapConnections() 
{
	int cellX, cellY;
	vertex* originVertex;
	vertex* neighbour;
	int neighbourY, neighbourX;
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
		
		         //If the cell is not a BLOCKED CELL
		         if(map[j][i].type != '1'){
		
						originVertex = &map[j][i];
						for(int m=0; m < DIRECTIONS; m++){
							neighbour = map[j][i].move[m];
							if(neighbour != NULL && neighbour->type != '1'){
								//for debugging only
								//cout << "cost[" << m << "] = " << originVertex->linkCost[m] << endl; 
								
								setcolor(YELLOW);
								setlinestyle(WIDE_DOT_FILL, 2, 2);
							   line(neighbour->centre.x, neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
							}  else if(neighbour->type == '1'){
								setcolor(RED);
								setlinestyle(WIDE_DOT_FILL, 2, 2);
							   line(neighbour->centre.x, neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
								//for debugging only
								//cout << "cost[" << m << "] = " << originVertex->linkCost[m] << endl; //<< "blocked cell" << endl; 
								
							}
							
						}
				   } 
					
					
					//~ if(map[j][i].type == '0') //traversable cell
					//~ {						
						//~ //markCell_col_row(i,j, LIGHTGRAY, WHITE);
						
					//~ }
					
					//~ if(map[j][i].type == '1') //'B' - blocked cell
					//~ {
						//~ //markCell_col_row(i,j, BLACK, WHITE);
						
					//~ }
					//~ if(map[j][i].type == '9') //unknown
					//~ {
						//~ //markCell_col_row(i,j, DARKGRAY, WHITE);
						
					//~ }
					//~ if(map[j][i].type == '6') //'S' - start vertex
					//~ {
						//~ startVertex.row = j;
						//~ startVertex.col = i;
						//~ //markCell_col_row(i,j, GREEN, WHITE);
						
					//~ }
					//~ if(map[j][i].type == '7') //'G' - goal vertex
					//~ {
						//~ goalVertex.row = j;
						//~ goalVertex.col = i;
						//~ //markCell_col_row(i,j, BLUE, WHITE);
						
					//~ }
					
				} //for next Cols
			} //for next Rows
		setlinestyle(SOLID_LINE, 1, 1);
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception dispaying Map\n";
   }
  
 
  
}


void GridWorld::displayMap() 
{
	int cellX, cellY;
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					bool drawBobot = false;
					DstarLiteCell *lastStart = getDstarLiteLastStart();
					if (lastStart && ALGORITHM == DSTARLITE) // Draw robot
					{
						if (lastStart->y == j && lastStart->x == i)
							drawBobot = true;
					}
					if(map[j][i].type == '0') //traversable cell
					{	

						if (showExpandedCells && map[j][i].expanded)
						{
							markCell_col_row(i, j, YELLOW, WHITE, drawBobot);
						}
						else {
							markCell_col_row(i, j, LIGHTGRAY, WHITE, drawBobot);
						}
						
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						markCell_col_row(i, j, BLACK, WHITE, drawBobot);
					}
					if(map[j][i].type == '9') //unknown
					{
						markCell_col_row(i, j, 47, WHITE, drawBobot);

					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row(i,j, GREEN, WHITE, drawBobot);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row(i,j, BLUE, WHITE, drawBobot);
						
					}
					
					if(map[j][i].type == '4') //'SUB-GOAL'
					{
						map[j][i].centre = markCell_col_row(i,j, 47, WHITE, drawBobot);
						
					}
					
				}
			}
		
		
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception dispaying Map\n";
   }
  
 
  
}

void GridWorld::displayMapWithPositionDetails() 
{
	int cellX, cellY;
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						markCell_col_row_details_xy(i,j, LIGHTGRAY, WHITE);
						
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						markCell_col_row_details_xy(i,j, BLACK, WHITE);
						
					}
					if(map[j][i].type == '9') //unknown
					{
						markCell_col_row_details_xy(i,j, 47, WHITE);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row_details_xy(i,j, GREEN, WHITE);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details_xy(i,j, BLUE, WHITE);
						
					}
					
				}
			}
		
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception dispaying Map\n";
   }
}

void GridWorld::displayMapWithDetails() 
{
	int cellX, cellY;
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						//markCell_col_row_details(i,j, LIGHTGRAY, WHITE);
						markCell_col_row_details(i,j, LIGHTGRAY, WHITE);
						
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						//markCell_col_row_details(i,j, BLACK, WHITE);
						markCell_col_row_details(i,j, BLACK, WHITE);
						
					}
					if(map[j][i].type == '9') //unknown
					{
						markCell_col_row_details(i,j, 47, WHITE);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row_details(i,j, GREEN, WHITE);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details(i,j, BLUE, WHITE);
						
					}
					
					if(map[j][i].type == '4') //SUB GOAL
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details(i,j, 47, WHITE);
						
					}
					
				}
			}
		
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception dispaying Map\n";
   }
}


//~ void GridWorld::displayMapWithRLDetails(bool display_reward, bool display_maxQ, long maxEpoch) 
//~ {
	//~ int cellX, cellY;
	
		
	//~ try{
		//~ //---------------------------------------
		 //~ //displayHeader();
		 //~ displayRLHeader(maxEpoch);
		//~ //---------------------------------------		
			//~ for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			//~ {
				
				//~ for(int i =0;i < GRIDWORLD_COLS; i++) //col
				//~ {
					
					//~ if(map[j][i].type == '0') //traversable cell
					//~ {						
						//~ markCell_col_row_RL_details(i,j, LIGHTGRAY, WHITE, display_reward, display_maxQ);
					//~ }
					
					//~ if(map[j][i].type == '1') //'B' - blocked cell
					//~ {
						//~ markCell_col_row_RL_details(i,j, BLACK, WHITE, display_reward, display_maxQ);
						
					//~ }
					//~ if(map[j][i].type == '9') //unknown
					//~ {
						//~ markCell_col_row_RL_details(i,j, DARKGRAY, WHITE, display_reward, display_maxQ);
						
					//~ }
					//~ if(map[j][i].type == '6') //'S' - start vertex
					//~ {
						//~ startVertex.row = j;
						//~ startVertex.col = i;
						//~ markCell_col_row_RL_details(i,j, GREEN, WHITE, display_reward, display_maxQ);
						
					//~ }
					//~ if(map[j][i].type == '7') //'G' - goal vertex
					//~ {
						//~ goalVertex.row = j;
						//~ goalVertex.col = i;
						//~ markCell_col_row_RL_details(i,j, BLUE, WHITE, display_reward, display_maxQ);
						
					//~ }
					//~ if(map[j][i].type == '4') //sub GOAL
					//~ {
						//~ goalVertex.row = j;
						//~ goalVertex.col = i;
						//~ markCell_col_row_RL_details(i,j, CYAN, WHITE, display_reward, display_maxQ);
						
					//~ }
					
				//~ }
			//~ }
		
	//~ }
   //~ catch (std::ifstream::failure e) {
    //~ std::cerr << "Exception dispaying Map\n";
   //~ }
//~ }

void GridWorld::displayMapWithKeyDetails() 
{
	int cellX, cellY;
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						markCell_col_row_details(i,j, LIGHTGRAY, WHITE, true, true, false, true);
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						markCell_col_row_details(i,j, BLACK, WHITE, true, true, false, true);
						
					}
					if(map[j][i].type == '9') //unknown
					{
						markCell_col_row_details(i,j, 47, WHITE, true, true, false, true);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row_details(i,j, GREEN, WHITE, true, true, false, true);
		
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details(i,j, BLUE, WHITE, true, true, false, true);
						
					}

				}
			}
		
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception dispaying Map\n";
   }
}


void GridWorld::displayMapWithSelectedDetails(bool display_g, bool display_rhs, bool display_h, bool display_key) 
{
	int cellX, cellY;
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						markCell_col_row_details(i,j, LIGHTGRAY, WHITE, display_g, display_rhs, display_h, display_key);
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						markCell_col_row_details(i,j, BLACK, WHITE, display_g, display_rhs, display_h, display_key);
						
					}
					if(map[j][i].type == '9') //unknown
					{
						markCell_col_row_details(i,j, 47, WHITE, display_g, display_rhs, display_h, display_key);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row_details(i,j, GREEN, WHITE, display_g, display_rhs, display_h, display_key);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details(i,j, BLUE, WHITE, display_g, display_rhs, display_h, display_key);	
					}
				}
			}
		
	}
   catch (std::ifstream::failure e) {
    std::cerr << "Exception dispaying Map\n";
   }
}



void GridWorld::initSystemOfCoordinates(){
   

	 WORLD_MAXX = 220.0f;
    WORLD_MAXY = 180.0f;
	
	
	
	
	 fieldX1 = getmaxx() / 10;
    fieldX2 = getmaxx() - (getmaxx() / 10);
    fieldY1 = getmaxy() / 6; //top of the screen
    fieldY2 = getmaxy() - (getmaxy() / 10); //bottom of the screen
	 
	 cout << "Device Coordinates(" << fieldX1 << ", " << fieldY1 << ", " << fieldX2 << ", " << fieldY2 << ")\n";

//--------------------------------------------------------	
		    
//5 vs 5 MIROSOT FIELD    
	//World boundaries
    worldBoundary.x1 = 0.0;
    worldBoundary.y1 = WORLD_MAXY;
    worldBoundary.x2 = WORLD_MAXX;
    worldBoundary.y2 = 0.0;

	//Device boundaries
    deviceBoundary.x1 = fieldX1;
    deviceBoundary.y1 = fieldY1;
    deviceBoundary.x2 = fieldX2;
    deviceBoundary.y2 = fieldY2;
	 	
}
///////////////////////////////////////////////////////////////////////////

void GridWorld::drawGrid(){
	int countVertLines, countHorizLines;
	
	countVertLines=0;
	countHorizLines=0;
	
	
   setlinestyle(WIDE_DOT_FILL, 1, 1);
   //setcolor(DARKGRAY);
	
   
	setcolor(LINE_COLOUR);
   //draw vertical bars
	
	
	//for(int x=deviceBoundary.x1; x <= deviceBoundary.x2; x += robotWidth){
	for(int x=deviceBoundary.x1; x <= deviceBoundary.x2; x += xInc){
		line(x, fieldY1, x, fieldY2);
	}
	
	//for(int y=deviceBoundary.y1; y <= deviceBoundary.y2; y += robotWidth){
	for(int y=deviceBoundary.y1; y <= deviceBoundary.y2; y += yInc){
		line(fieldX1, y, fieldX2, y);
	}
   	
	
	//setcolor(WHITE);
	//rectangle(fieldX1, fieldY1, fieldX2, fieldY2);
	
	//cout << "drawGrid complete." << endl;
}

void GridWorld::markCell(int x, int y){
	
	int cellX, cellY;
	

	cellX = deviceBoundary.x1 + (((x-deviceBoundary.x1)/cellWidth) * cellWidth);
	cellY = deviceBoundary.y1 + (((y-deviceBoundary.y1)/cellHeight) * cellHeight);
	
	setcolor(BLUE);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
}

CellPosition GridWorld::getCellPosition_markCell(int x, int y){
	
	int cellX, cellY;
	CellPosition p;

	cellX = deviceBoundary.x1 + (((x-deviceBoundary.x1)/cellWidth) * cellWidth);
	cellY = deviceBoundary.y1 + (((y-deviceBoundary.y1)/cellHeight) * cellHeight);
	
	setcolor(BLUE);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
	int col = (int(x-deviceBoundary.x1)/cellWidth)+1;
	int row = (int(y-deviceBoundary.y1)/cellHeight)+1;
	
	p.col = col;
	p.row = row;
	return p;
	
}

int GridWorld::getGridMaxX(){
	if( (cellWidth < 0) || (GRIDWORLD_COLS < 0) ){
		cout << "Error in GridWorld::getGridMaxX()" << endl;
		exit(-1);
	}
	//return (deviceBoundary.x1 + ((GRIDWORLD_COLS-1) * cellWidth));
	return (deviceBoundary.x1 + ((GRIDWORLD_COLS) * cellWidth));
}

int GridWorld::getGridMaxY(){
	if( (cellHeight < 0) || (GRIDWORLD_ROWS < 0) ){
		cout << "Error in GridWorld::getGridMaxy()" << endl;
		exit(-1);
	}
	//return (deviceBoundary.y1 + ((GRIDWORLD_ROWS-1) * cellHeight));
	return (deviceBoundary.y1 + ((GRIDWORLD_ROWS) * cellHeight));
}

Coordinates GridWorld::markCell_col_row(int col, int row, int fillColour, int outlineColour, bool showRobot){
	
	int cellX, cellY;
	
 	if(col == 0){
		cellX = deviceBoundary.x1;
	} else {
	   //cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
		cellX = deviceBoundary.x1 + ((col) * cellWidth);
	}
	
	if(row == 0){
	   cellY = deviceBoundary.y1;
	} else {
		//cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
		cellY = deviceBoundary.y1 + ((row) * cellHeight);
	}
	
	//~ cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
	//~ cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
    setcolor(fillColour);
	setfillstyle(SOLID_FILL, fillColour);
	bar(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	setcolor(outlineColour);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
	Coordinates c;

	
	c.x = (cellX + cellX+cellWidth)/2;
	c.y = (cellY + cellY+cellHeight)/2;
	
	if (showRobot)
	{
		setcolor(RED);
		circle(c.x, c.y, cellWidth / 4);
	}
	else {
		circle(c.x, c.y, cellWidth / 12);
	}

	
	
	return c;
}

void GridWorld::markCell_col_row_details(int col, int row, int fillColour, int outlineColour, bool display_g, bool display_rhs, bool display_h, bool display_key){
	
	int cellX, cellY;
	char info[6];
	int x,y;
	

	if(col == 0){
		cellX = deviceBoundary.x1;
	} else {
	   //cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
		cellX = deviceBoundary.x1 + ((col) * cellWidth);
	}
	
	if(row == 0){
	   cellY = deviceBoundary.y1;
	} else {
		cellY = deviceBoundary.y1 + ((row) * cellHeight);
	}


   setcolor(fillColour);
	setfillstyle(SOLID_FILL, fillColour);
	setbkcolor(fillColour);
	bar(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	setcolor(WHITE);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
	//--------------------------------------------
	y = cellY; 

	settextstyle(SMALL_FONT, HORIZ_DIR, 4);
	settextjustify(LEFT_TEXT,TOP_TEXT);
	//setcolor(WHITE);
	//setcolor(outlineColour);
	setcolor(RED);
	
	if(display_g){
		x = cellX + textwidth("I");
		y = y + textheight(";");
		
		double g = map[row][col].g;
		
		if(g > (INF - 0.0001)){ //tolerance = 0.0001
			sprintf(info,"g = INF");
		} else {
			sprintf(info,"g = %3.1f",g);
		}
		setbkcolor(fillColour);
		outtextxy(x ,y, info);
   }

	//--------------------------------------------
	
	if(display_rhs){
		x = cellX + textwidth("I");
		y = y + textheight(";");
		
		double rhs = map[row][col].rhs;
		
		if(rhs > (INF - 0.0001)){ //tolerance = 0.0001
			sprintf(info,"rhs = INF");
		} else {
			sprintf(info,"rhs = %3.1f",rhs);
		}
		setbkcolor(fillColour);
		outtextxy(x ,y, info);
   }

	//--------------------------------------------
		
	if(display_h){
		x = cellX + textwidth("I");
		y = y + textheight(";");
		double h = map[row][col].h;
		
		if(h > (INF - 0.0001)){ //tolerance = 0.0001
			sprintf(info,"h = INF");
		} else {
			sprintf(info,"h = %3.1f",h);
		}
		setbkcolor(fillColour);
		outtextxy(x ,y, info);
   }

	//--------------------------------------------
		
	if(display_key){
		char info1[6];
		char info2[6];
		x = cellX + textwidth("I");
		y = y + textheight(";");
		double k1 = map[row][col].key[0];
		double k2 = map[row][col].key[1];
		
		if(k1 > (INF - 0.0001)){ //tolerance = 0.0001
			//sprintf(info1,"k1 = INF");
			sprintf(info1,"INF");
		} else {
			//sprintf(info1,"k1 = %3.1f",k1);
			sprintf(info1,"%3.1f",k1);
		}
		
		if(k2 > (INF - 0.0001)){ //tolerance = 0.0001
			//sprintf(info2,"k2 = INF");
			sprintf(info2,"INF");
		} else {
			//sprintf(info2,"[k2 = %3.1f",k2);
			sprintf(info2,"%3.1f",k2);
		}
		
		sprintf(info,"[%s, %s]",info1, info2);
		
		setbkcolor(fillColour);
		outtextxy(x ,y, info);
   }

}




void GridWorld::markCell_col_row_details_xy(int col, int row, int fillColour, int outlineColour){
	
	int cellX, cellY;
	char info[6];
	int x,y;
	
	if(col == 0){
		cellX = deviceBoundary.x1;
	} else {
	   //cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
		cellX = deviceBoundary.x1 + ((col) * cellWidth);
	}
	
	if(row == 0){
	   cellY = deviceBoundary.y1;
	} else {
		//cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
		cellY = deviceBoundary.y1 + ((row) * cellHeight);
	}
	

	//~ cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
	//~ cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
	
   setcolor(fillColour);
	setfillstyle(SOLID_FILL, fillColour);
	setbkcolor(fillColour);
	bar(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	setcolor(outlineColour);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
	//--------------------------------------------
			
	y = cellY; // + textheight("_");
		
	settextstyle(SMALL_FONT, HORIZ_DIR, 4);
	settextjustify(LEFT_TEXT,TOP_TEXT);
	setcolor(WHITE);
	
	
	
	
	//--------------------------------------------
		
	
	char info1[6];
	char info2[6];
	x = cellX + textwidth("I");
	y = y + textheight(";");
	int cellCol = map[row][col].col;
	int cellRow = map[row][col].row;
	
	if(cellCol == INF){ 
		sprintf(info1,"INF");
	} else {
		sprintf(info1,"%d",cellCol);
	}
	
	if(cellRow == INF){ 
		sprintf(info2,"INF");
	} else {
		sprintf(info2,"%d",cellRow);
	}
	
	sprintf(info,"col=%s,row=%s",info1, info2);
	
	setbkcolor(fillColour);
	outtextxy(x ,y, info);

}


void GridWorld::markCell_col_row_details(int col, int row, int fillColour, int outlineColour, bool showRobot){
	
	int cellX, cellY;
	
	if(col == 0){
		cellX = deviceBoundary.x1;
	} else {
	   //cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
		cellX = deviceBoundary.x1 + ((col) * cellWidth);
	}
	
	if(row == 0){
	   cellY = deviceBoundary.y1;
	} else {
		//cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
		cellY = deviceBoundary.y1 + ((row) * cellHeight);
	}
	

	//~ cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
	//~ cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
	
   setcolor(fillColour);
	setfillstyle(SOLID_FILL, fillColour);
	setbkcolor(fillColour);
	bar(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	setcolor(outlineColour);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	//--------------------------------------------
	int x,y;
	
	x = cellX + textwidth("I");
	y = cellY + textheight("_");
	//y = cellY; //  + textheight(";");
	settextstyle(SMALL_FONT, HORIZ_DIR, 5);
	settextjustify(LEFT_TEXT,TOP_TEXT);
	setcolor(WHITE);
	
	char info[6];
	double g = map[row][col].g;
	
	if(g > (INF - 0.0001) && g < (INF + 0.0001)){ //tolerance = 0.0001
		sprintf(info,"g = INF");
	} else {
	   sprintf(info,"g = %3.1f",g);
	}
	setbkcolor(fillColour);
	outtextxy(x ,y, info);
	//--------------------------------------------
	y = y + textheight(";");
	
	double rhs = map[row][col].rhs;
	
	if(rhs > (INF - 0.0001) && rhs < (INF + 0.0001)){ //tolerance = 0.0001
		sprintf(info,"rhs = INF");
	} else {
	   sprintf(info,"rhs = %3.1f",rhs);
	}
	setbkcolor(fillColour);
	outtextxy(x ,y, info);
	//--------------------------------------------
	y = y + textheight(";");
	
	double h = map[row][col].h;
	
	if(h > (INF - 0.0001) && h < (INF + 0.0001)){ //tolerance = 0.0001
		sprintf(info,"h = INF");
	} else {
	   sprintf(info,"h = %3.1f",h);
	}
	setbkcolor(fillColour);
	outtextxy(x ,y, info);

	Coordinates c;

	c.x = (cellX + cellX + cellWidth) / 2;
	c.y = (cellY + cellY + cellHeight) / 2;

	if (showRobot)
	{
		circle(c.x, c.y, cellWidth / 4);
	}
	
}

