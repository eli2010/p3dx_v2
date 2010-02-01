#ifndef _BALLDETECTION_H_
#define _BALLDETECTION_H_
#include "ArRobot.h"
#include "Aria.h"
#include "irpImage/UniImageAll.h"
#include "irpVideo/Grabber.h"
#include "SkinDetection.h"


using namespace std;
/* Variablen für die Bildgröße (höhe, breite)
*  und Größe des Fensters mit welchem der Ball gesucht wird
*/
const int heigth=600;
const int width=800;
const int windowSize=100;



class ballDetection {
public:
	/** 
	* Initialisierungs Methode für die Balldetection
	* Es muss lediglich ein "ballDetection" Objekt erzeugt werden, diese verbindet
	* dann zur kamera und berechnet anhand eines LUT (Lookuptable) die Wahrscheinlichkeit
	* ob im aktuellen Bild ein ball vorhanden ist.
	* Das Bild wird mit einem 100*100 (see windowSize) Fenster durchlaufen.
	* Sobald in diesem Fenster ein gewisser Schwellwert überschritten wird
	* werden grüne Pixel mit minimale x und y Koordinate, MinX und MaxY Koordinate, 
	* MaxX und MinY Koordinate, MaxX und MaxY Koordinate berechnet. innerhalb dieses 
	* neuen Fensters wird der prozentuale Anteil der grünen Punkte berechnet. Dies
	* dient der Unterscheidung zwischen Ball und Blume zB. Sobald der Roboter allerdings 
	* sehr nah am Objekt ist, schlägt die erkennung fehl.
	*
	* @param p3dx - Pointer auf den Roboter
	*/
	ballDetection(ArRobot *p3dx);
	/**
	* moveToBall sobald der Schwellwert überschritten wurde dreht diese Methode den Roboter
	* Solange bis der Ball in der Mitte ist (mithilfe der rotateToBall() Methode
	*
	*/
	int moveToBall();
	/**
	* Diese Methode gibt true zurück sobald der ball in der Mitte des Kamerabildes ist
	*/
	int rotateToBall();
	/**
	* Diese Methode beendet das Programm sobald der Roboter den Ball erreicht hat
	*/
	void reached();
	


private:
	/**
	* die Methode Findobj() sucht mithilfe des 100*100 Fensters nach grünen Punkten
	* innerhalb des aktuellen Kamerabildes und gibt den höchsten Wert zurück
	*/
	double findObj();
	/* Initialisierung des Bildgrabbers */
	void initialGrabber();
	/** 
	* Diese Methode überprüft mit dem oben genannten 
	* Verfahren den prozentualen Anteil von grünen Pixeln 
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