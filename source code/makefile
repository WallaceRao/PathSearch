Main.exe		: 	Main.o  transform.o AstarSearch.o LPAstar.o dstarlite.o gridworld.o graphics.o 
	g++ -o Main.exe Main.o transform.o AstarSearch.o LPAstar.o dstarlite.o gridworld.o graphics.o -l gdi32 -g
			
Main.o		:	Main.cpp graphics.h transform.h AstarSearch.h LPAstar.h dstarlite.h gridworld.h globalvariables.h
	g++ -c -g -std=c++11    Main.cpp
	
transform.o		:	 transform.cpp transform.h
	g++ -c -g -std=c++11    transform.cpp	
	
AstarSearch.o	:	 AstarSearch.cpp AstarSearch.h
	g++ -c -g -std=c++11    AstarSearch.cpp

LPAstar.o	:	 LPAstar.cpp LPAstar.h
	g++ -c -g -std=c++11    LPAstar.cpp

dstarlite.o	:	 dstarlite.cpp dstarlite.h
	g++ -c -g -std=c++11    dstarlite.cpp

gridworld.o	:	 gridworld.cpp gridworld.h
	g++ -c -g -std=c++11    gridworld.cpp

graphics.o		:	 graphics.cpp graphics.h
	g++ -c -g -std=c++11    graphics.cpp
	
clean:
	del *.o
	del *.exe
