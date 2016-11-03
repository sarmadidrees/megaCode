
#ifndef AstarPathFinder_h
#define AstarPathFinder_h

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#if defined (ARDUINO) && ARDUINO >= 100
   #include <Arduino.h>
#else
   #include <WProgram.h>
   #include <pins_arduino.h>
#endif

#if defined (__AVR__)
   #include <avr/io.h>
#endif

#define mapX 11
#define mapY 5
#define extraX 4
#define extraY 1

#define pathSize 25


class AstarPathFinder
{
public:
	AstarPathFinder();
	void initNodes();		//initialize node
	void obstacle(int x, int y, bool walkable); 	//to add obstacle (true = is Walkable, false = not Walkable) 
	void findPath(int startY, int startX, int endY, int endX);			// to find path and store it into global struct "finalPath"
  	unsigned int stepCount();			//to find steps to reach final goal point
  	bool pathFound();		//to check whether the path is found correctly or not
  	void Flush();		//to reset all values
  	
	struct newPATH{
	    uint8_t X;	
	    uint8_t Y;
	    uint8_t F;
    }finalPath[pathSize];
  
private:
    bool _MAP[mapX + extraX][mapY + extraY];
    
	struct NODE{
	    bool isWalkable;
	    bool onOpenList;
	    bool onClosedList;
	    uint8_t parentX;
	    uint8_t parentY;
	    uint8_t G;
	    uint8_t H;
	    uint8_t F;
	  }_node[mapX + extraX][mapY + extraY];

    struct PATH{
	    uint8_t X;	
	    uint8_t Y;
	    uint8_t F;
    }_path[pathSize];

    int _x;
    int _y;

    uint8_t _currentX;
    uint8_t _currentY;
    uint8_t _startX;
    uint8_t _startY;
    uint8_t _endX;
    uint8_t _endY;
    uint8_t _parentG;

    int _count;

    uint8_t _stepCount;
    bool _pathFound;
	
};

#endif
