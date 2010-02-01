#ifndef _P3DXCONTROLS_H_
#define _P3DXCONTROLS_H_
#include "ArRobot.h"
#include "Aria.h"
#include <iostream>
#include<conio.h>
/**
* Dies Klasse dient der Steuerung und Bewegung des Roboters.
* Hier werden alle wichtigen Headerdatein eingebunden und zwischen Autonom fahren und 
* Tastatursteuerung umgeschaltet.
*/
#include <stdio.h> 
#include "mapping.h"
#include "ballDetection.h"

using namespace std;

const int secureDistance=500;
const int maxVel=75;
const int maxRotVel=15;


class p3dxControls {
public:
	/**
	* Konstruktur zum Erstellen der Steuerung. Nach dem Aufruf wird automatisch die Tastatursteuerung
	* gestartet und auf Tasteneingabe gewartet
	*/
	p3dxControls();
	p3dxControls(ArRobot *p3dx, mapping *map);
	void FindNarrowPassage();
	ArRobot *p3dxptr;
	mapping *mapptr;
private:
	/**
	* Diese Methode lässt den Roboter autonom durch einen raum fahren.
	*/
	void lookAround();
	/**
	* Hier wird zusätzlich zum autonomen Fahren noch eine Kamera benutzt welche einen Grünenball sucht
	* weiteres dazu in den Balldetection dokumentationen
	*/
	void lookAroundFindBall();
	/**
	* ToDo: diese Methode bestimmt einen engen Weg und sucht eine bestmögliche orientierung um den 
	* Roboter hindurch zu steuern
	*/
	void checkNarrow();
	/**
	* Überprüfung ob der Roboter sich in der nähe einer Wand befindet 
	*/
	
	void Control();
	
	double Rot_Value;
	double Vel_Value;
	char key;
};
#endif