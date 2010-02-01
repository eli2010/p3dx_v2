#ifndef _MOVEMENT_H_
#define _MOVEMENT_H_
#include "ArRobot.h"
#include "Aria.h"
#include <iostream>
#include<conio.h>
#include <stdio.h>

using namespace std;

const int secureDistance=500;
const int maxVel=75;
const int maxRotVel=15;

class movement {
public:
	movement(ArRobot *p3dx);
	void lookAround();
	void lookAroundFindBall();
	void FindNarrowPassage();
	void keyControl();
	
private:
	void checkNarrow();
	bool checkWall();
	ArRobot *p3dxptr;
};
#endif