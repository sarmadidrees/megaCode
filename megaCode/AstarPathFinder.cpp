
#include "AstarPathFinder.h"


//basic map with only walls and boxes on specific places
//1=walkable, 0=wall
bool MAP[mapX+extraX][mapY+extraY] = {{1,1,0,1,1,1},
							          {1,1,1,1,1,1},
							          {1,1,0,1,1,1},
							          {1,1,0,0,0,1},
							          {1,1,1,1,1,1},
							          {1,0,0,0,1,1},
							          {1,1,1,0,1,1},
							          {1,1,1,0,1,1},
							          {1,1,1,0,1,1},
							          {0,1,0,0,1,1},
							          {1,1,1,1,1,1},
							          {1,1,0,0,0,1},
							          {1,1,0,1,1,1},
							          {1,1,1,1,1,1},
							          {1,1,0,1,1,1}
							         };


/* static functions */
static unsigned int realY(int y){
	uint8_t yNew;
	yNew = y;

	if( yNew >= 2){
		yNew += 1;
	}

	return yNew;
}

static unsigned int realX(int x){
	uint8_t xNew;
	xNew = x;

	if ( xNew == 3) xNew += 1;
	else if (( xNew >= 4) && (xNew <= 6)) xNew += 2;
	else if (xNew == 7) xNew += 3;
	else if (( xNew >= 8) && (xNew <= 10)) xNew += 4;

	return xNew;
}
/* end of static functions*/


AstarPathFinder::AstarPathFinder(){
  	memcpy(_MAP,MAP, sizeof(_MAP));					//Copy global Array to private Array
}

void AstarPathFinder::initNodes(){

	AstarPathFinder::Flush();
 _pathFound = false;
 _stepCount = 0;
}

void AstarPathFinder::obstacle(int x, int y, bool walkable){
	_x = realX(x);
	_y = realY(y);
  
	if (walkable){
		_MAP[_x][_y] = true;
		_node[_x][_y].isWalkable = true;
	}
	else {
		_MAP[_x][_y] = false;
		_node[_x][_y].isWalkable = false;
	}

}

void AstarPathFinder::findPath(int startY, int startX, int endY, int endX){
  	
	int prevX = -1; 
	int prevY = -1;
	int sameCount = 0;

	_pathFound = false;		//initially make path found FALSE

	_startX = realX(startX);
	_startY = realY(startY);

	_endX = realX(endX);
	_endY = realY(endY);

	//make starting point as closed and end point as walkable 
    _node[_startX][_startY].onOpenList = false;
	_node[_startX][_startY].onClosedList = true;
	_node[_endX][_endY].isWalkable = true;

	_parentG = _node[_startX][_startY].G;

	_currentX = _startX;
	_currentY = _startY;

	//start finding path
  	//run path finding until it reaches end point
	while((_currentX != _endX) || (_currentY != _endY)){

	    //first find adjacent nodes wrt to current node
	    //adjacent nodes in horizontal
	    for(int i=-1;i<2;i+=2){
	        if((_currentY + i >= 0) && (_currentY + i < (mapY+extraY))){
	            if ((_node[_currentX][_currentY + i].isWalkable) && !(_node[_currentX][_currentY + i].onClosedList)){
                  
	                if( !(_node[_currentX][_currentY + i].onOpenList) ){
	                    _node[_currentX][_currentY + i].parentX = _currentX;
	                    _node[_currentX][_currentY + i].parentY = _currentY;
	                    _node[_currentX][_currentY + i].G = _parentG + 1;        
	                    _node[_currentX][_currentY + i].H = abs(_currentX - _endX) + abs(_currentY - _endY);
	                    _node[_currentX][_currentY + i].F = _node[_currentX][_currentY + i].G + _node[_currentX][_currentY + i].H;   //MANHATTAN method
                      _node[_currentX][_currentY + i].onOpenList = true;
	                }
	            }
	        }
	    }//end of adjacent nodes in horizontal

	    //adjacent nodes in vertical
	    for(int i=-1;i<2;i+=2){
	        if((_currentX + i >= 0) && (_currentX + i < (mapX+extraX))){
	            if ((_node[_currentX + i][_currentY].isWalkable) && !(_node[_currentX + i][_currentY].onClosedList)){

	                if( !(_node[_currentX + i][_currentY].onOpenList) ){
	                    _node[_currentX + i][_currentY].parentX = _currentX;
	                    _node[_currentX + i][_currentY].parentY = _currentY;
	                    _node[_currentX + i][_currentY].G = _parentG + 1;        
	                    _node[_currentX + i][_currentY].H = abs(_currentX - _endX) + abs(_currentY - _endY);
	                    _node[_currentX + i][_currentY].F = _node[_currentX + i][_currentY].G + _node[_currentX + i][_currentY].H;   //MANHATTAN method
	                    _node[_currentX + i][_currentY].onOpenList = true;
	                }
	            }
	        }
	    }//end of adjacent nodes in vertical

	    int lowestF = 500;
	    int lowestG = 500;

	    //check for lowest F in open list nodes
	    //and if more than 2 have same F than decide on its G value
	    for(uint8_t x=0; x<(mapX+extraX); x++){
	        for(uint8_t y=0; y<(mapY+extraY); y++){
	            if((_node[x][y].onOpenList) && !(_node[x][y].onClosedList)){
	                if(_node[x][y].F < lowestF){
	                    _currentX = x;
	                    _currentY = y;
	                    lowestF = _node[x][y].F;
	                    lowestG = _node[x][y].G;
	                }
	                else if ( (_node[x][y].F == lowestF) && (_node[x][y].G < lowestG)){
	                    _currentX = x;
	                    _currentY = y;
	                    lowestF = _node[x][y].F;
	                    lowestG = _node[x][y].G;
	                }
	            }
	        }
	    }

	    //if path is not found correctly then it should terminate the procedure
	    if ((prevX == _currentX) && (prevY == _currentY)){
        	sameCount++;

        	if (sameCount >= 5) {
    			_pathFound = false;
    			return;
    		}
    	}

	    //make current node as closed
	    _node[_currentX][_currentY].onClosedList = true;
	    _node[_currentX][_currentY].onOpenList = false;

	    _parentG = _node[_currentX][_currentY].G;

	    //store previous values to keep check that it is not running in a while loop forever!! 
	    prevX = _currentX;
    	prevY = _currentY;
	}
  
	  _currentX = _endX;
    _currentY = _endY;

    _count = 0;
    _path[_count].X = _endX;		//make first path as end node
    _path[_count].Y = _endY;
    _count++;

    //to store the found path, start from ending point and get all parent node back to the starting point
    while((_currentX != _startX) || (_currentY != _startY)){

        _path[_count].X = _node[_currentX][_currentY].parentX;
        _path[_count].Y = _node[_currentX][_currentY].parentY;
        _path[_count].F = _node[_currentX][_currentY].F;
        _currentX = _path[_count].X;
        _currentY = _path[_count].Y;
        _count++;
    }

    _count -= 1;
    uint8_t i = 0;
    
    //Final Path for the REAL arena
   	while (_count > -1){
  
   		if((_path[_count].Y == 2) || (_path[_count].X == 3) || (_path[_count].X == 5) || (_path[_count].X == 9) || (_path[_count].X == 11)){
   			_count -= 1;
   		}
   		else {
  
   				if(_path[_count].Y >= 3){
   					finalPath[i].Y = _path[_count].Y - 1;
   				}
   				else
   					finalPath[i].Y = _path[_count].Y;
  
   				if ( _path[_count].X == 4) 
   					 	finalPath[i].X = _path[_count].X - 1;
  					else if (( _path[_count].X >= 5) && (_path[_count].X <= 9)) 
  						finalPath[i].X = _path[_count].X - 2;
  					else if (_path[_count].X == 10) 
  						finalPath[i].X = _path[_count].X - 3;
  					else if (( _path[_count].X >= 11) && (_path[_count].X <= 14)) 
  						finalPath[i].X = _path[_count].X - 4;
  					else 
  						finalPath[i].X = _path[_count].X;
  
  				_count -= 1;
  				i++;
   		}
   	}

     _stepCount = i - 1;		//store steps of path into private member
     _pathFound = true;		
}
 
unsigned int AstarPathFinder::stepCount(){
  return _stepCount;
}

bool AstarPathFinder::pathFound(){
	return _pathFound;
}

void AstarPathFinder::Flush(){

	//make all values to zero
	for (int x=0; x<(mapX+extraX); x++){
        for(int y=0; y<(mapY+extraY); y++){
            _node[x][y].isWalkable = true;
            _node[x][y].onOpenList = false;
            _node[x][y].onClosedList = false;
            _node[x][y].G = 0;
            _node[x][y].H = 0;
            _node[x][y].F = 0;
            _node[x][y].parentX = 0;
            _node[x][y].parentY = 0;
        }
    }

    for (int i=0; i<pathSize; i++ ){
    	_path[i].X = 0;
    	_path[i].Y = 0;
    	_path[i].F = 0;
    }

    for (int i=0; i<pathSize; i++ ){
    	finalPath[i].X = 0;
    	finalPath[i].Y = 0;
    	finalPath[i].F = 0;
    }

    //find isWalkable for every node
    for(int x=0; x<(mapX+extraX); x++){
        for(int y=0; y<(mapY+extraY); y++){
            if (_MAP[x][y] == false) {
              	_node[x][y].isWalkable = false;
              }
              else if (_MAP[x][y] == true)
              {
              	_node[x][y].isWalkable = true;	
              }
        }
    }
    //end finding isWalkable for every node

}
