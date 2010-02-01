#include <iostream>
#include<conio.h>
#include <stdio.h>
#include "p3dxControls.h"

using namespace std;
/* Robo Wlan addresse: 134.169.36.169
*  http://www.yolinux.com/TUTORIALS/LinuxTutorialC++STL.html#LIST
*  Generelle Idee zum optimieren des Mapping:
*  struct mit x,y,th (robo Posen), ArSensorReading zum Speichern der entsprechenden Readings zur Robo Pose
*  diese struct wird in list gespeichert und für jede messung werden die daten gespeichert.
*/

ArRobot p3dx;
mapping mainmapping(&p3dx);
int main(int argc, char** argv) {
	//* Initialisierungs Aufrufe des Roboters */
	ArSonarDevice Sonar;
	Aria::init();
	ArArgumentParser parser(&argc, argv);
	ArSimpleConnector sc(&parser);
	parser.loadDefaultArguments();
	Aria::parseArgs();
	p3dx.addRangeDevice(&Sonar);
	sc.connectRobot(&p3dx);
	p3dx.runAsync(true);
	p3dx.lock();
	p3dx.enableMotors();
	p3dx.enableSonar();
	p3dx.unlock();
	///* Sleep damit alle Änderungen vom Roboter registriert werden */
	ArUtil::sleep(2000);
	p3dx.comInt(ArCommands::SETV, 200);
	p3dx.comInt(ArCommands::SETRV, 50);
	ArPose p3dxPose = p3dx.getPose();
	
	/* Aufruf der Steuerungsmethode */ 
	new p3dxControls(&p3dx, &mainmapping);
	//mainmapping.testMethods();
}