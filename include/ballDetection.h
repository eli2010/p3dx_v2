#ifndef _BALLDETECTION_H_
#define _BALLDETECTION_H_
#include "ArRobot.h"
#include "Aria.h"
#include "irpImage/UniImageAll.h"
#include "irpVideo/Grabber.h"
#include "SkinDetection.h"


using namespace std;
/* Variablen f�r die Bildgr��e (h�he, breite)
*  und Gr��e des Fensters mit welchem der Ball gesucht wird
*/
const int heigth=600;
const int width=800;
const int windowSize=100;



class ballDetection {
public:
	/** 
	* Initialisierungs Methode f�r die Balldetection
	* Es muss lediglich ein "ballDetection" Objekt erzeugt werden, diese verbindet
	* dann zur kamera und berechnet anhand eines LUT (Lookuptable) die Wahrscheinlichkeit
	* ob im aktuellen Bild ein ball vorhanden ist.
	* Das Bild wird mit einem 100*100 (see windowSize) Fenster durchlaufen.
	* Sobald in diesem Fenster ein gewisser Schwellwert �berschritten wird
	* werden gr�ne Pixel mit minimale x und y Koordinate, MinX und MaxY Koordinate, 
	* MaxX und MinY Koordinate, MaxX und MaxY Koordinate berechnet. innerhalb dieses 
	* neuen Fensters wird der prozentuale Anteil der gr�nen Punkte berechnet. Dies
	* dient der Unterscheidung zwischen Ball und Blume zB. Sobald der Roboter allerdings 
	* sehr nah am Objekt ist, schl�gt die erkennung fehl.
	*
	* @param p3dx - Pointer auf den Roboter
	*/
	ballDetection(ArRobot *p3dx);
	/**
	* moveToBall sobald der Schwellwert �berschritten wurde dreht diese Methode den Roboter
	* Solange bis der Ball in der Mitte ist (mithilfe der rotateToBall() Methode
	*
	*/
	int moveToBall();
	/**
	* Diese Methode gibt true zur�ck sobald der ball in der Mitte des Kamerabildes ist
	*/
	int rotateToBall();
	/**
	* Diese Methode beendet das Programm sobald der Roboter den Ball erreicht hat
	*/
	void reached();
	


private:
	/**
	* die Methode Findobj() sucht mithilfe des 100*100 Fensters nach gr�nen Punkten
	* innerhalb des aktuellen Kamerabildes und gibt den h�chsten Wert zur�ck
	*/
	double findObj();
	/* Initialisierung des Bildgrabbers */
	void initialGrabber();
	/** 
	* Diese Methode �berpr�ft mit dem oben genannten 
	* Verfahren den prozentualen Anteil von gr�nen Pixeln 
	* innerhalb einer BoundingBox 
	*/
	int checkball();
	/* Diverse Objekte zur Bilderkennung und Verarbeitung */
	ArRobot *p3dxptr;
	int KoordX;
	int KoordY;
	img::ByteRGBImage rgb;
	img::ByteRGBImageDialog rgbdlg;

	vid::Grabber grabber;
	vid::Grabber::Configuration grabconfig;
	face::SkinDetection detec;
	vector<img::ByteRGBImage> oldpics;
	vector<img::ByteRGBImage> refpics;
};

#endif