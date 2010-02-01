#ifndef _MAPPING_H_
#define _MAPPING_H_
#include "ArRobot.h"
#include "Aria.h"
#include <iostream>
#include<conio.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits>
#include "irpImage/UniImageAll.h"
#include "irpMath/Mat.h"
#include "OccupancyGridMap.h"
#include "pthread.h"
//#include "stocc.h"
#include <deque>
#define NUM_THREADS 1
#define SONAR_ANGLE 30.0
#define OBSTACLE_RANGE 3000.0

using namespace std;

const double res = 20.0;
const int size = 512;
const int anzsensor = 16;
const double offset_x = 256;
const double offset_y = 256;
const double headingOffset = 90;
const double ALPHA = 270*(M_PI/180);
class mapping {
public:
	mapping(ArRobot *p3dx);
	void runMapping();
	void startMapping();
	img::ByteImage b;
	img::ByteRGBImage bild;
	img::ByteRGBImage paarebild;
	img::ByteRGBImage demobild;
	img::ByteRGBImage paare;
	void groupMeasures();
	void ManuellMapping();
	//void groupMeasures(deque<measurement> *orig, deque<measurement> *grouped);
	void testMethods();
	void visualisieren();
	void matching();
	void pseudoMove();
	void map();
	
private:
	int anzMessungen;
	img::ByteImageDialog dlg;
	img::ByteRGBImageDialog bilddlg;
	img::ByteRGBImageDialog paardlg;
	img::ByteRGBImageDialog demodlg;
	ArPose *SonarGlobalPose;
	ArPose *SonarLocalPose;
	OccupancyGridMap *karte;
	pthread_t threads[NUM_THREADS];
	ArRobot *p3dxptr;
	struct measure {

		double range[3];
		double measure_cov[9];

	};
	struct measurement {
		std::deque<measure> SonarMeasures;
		ArPose roboPose;
		double robo_vec[3];
		double robo_cov[9];
	};
	struct pair {
		//speichern der C(i,j) Matrix
		//speichern des h Vektors
		//j_3 speichern
		double rob_pos_a[3];
		double rob_pos_b[3];
		double a_vec[3];
		double b_vec[3];
		double mahalaDis;
		math::Mat<2,2,double> C;
		math::Mat<2,2,double> C_1;
		math::Mat<2,3,double> J;
		math::Mat<3,2,double> J_T;
		math::Vec<2,double> h;
		
	};
	deque<measurement> messungen;
	deque<measurement> groupedMeasures;
	deque<measurement> Scan_One, Scan_Two, Scan_Three;
	deque<pair> measurePairs;
	void groupMeasures(deque<measurement> *orig, deque<measurement> *grouped);
	/**
	* Update der Lokalen SonarPosition
	*/
	void updateSonarLocalPose();
	/**
	* update der Globalen SonarPosition
	*/
	void updateSonarGlobalPose();
	/**
	* Diese Methode berechnet die entsprechenden SonarKegel für die OccupancyGridmap
	*/
	void calcSonar(int SensorID);
	/**
	*	Diese Methode prüft ob ein Objekt in der Naehe ist
	* @return true/false
	*/
	bool obstacleInRange();
	/**
	* Speichern von Roboterpositionen, Entfernungen von Objekten und generieren der initialen Kovarianzmatrizen
	*/
	void saveMeasure(ArRobot *robo, deque<measurement> *m);
	/**
	* Berechnung der Kovarianzmatrix zwischen RoboPos_1 und RoboPos_2
	* gespeichert wird in *calcedCov
	*/
	void calcCov(struct measurement *RoboPos_1, struct measurement *RoboPos_2, double *calcedCov);
	/**
	* Berechnung der Kovarianzmatrix zwischen einer Messung und einer Roboterbewegung
	* gespeichert wird in calcedCov
	*/
	void calcCov(struct measure *m, struct measurement *roboTrans, double *calcedCov);
	/**
	* Transponieren einer matrix *m
	*/
	void matrixTrans(math::Mat<3,3,double> *m, math::Mat<3,3,double> *m_erg);
	/**
	* Invertieren einer Transformation/Messung
	*/
	void InvTrans(struct measurement *Robo_Vec, double *erg_cov, double *erg_vec);
	/**
	* Berechnung eines Invertierten Transformations Vektors
	* gespeichert wird in *v_erg
	*/
	void calcInvVec(double *v, double *v_erg);
	/**
	* s.o. allerdings wird direkt in *v gespeichert
	*/
	void calcInvVec(double *v);
	/**
	* visualisiert eine deque von Messungen
	*/
	void visualMeasures(deque<measurement> *vec, img::ByteRGBImage *bild, img::ByteRGBImageDialog *dlg, int red =1);
	void visualRoboPose(measurement *m, img::ByteRGBImage *b, img::ByteRGBImageDialog *d, int red = 1);
	/**
	* Ausgabe von Messungen
	*/
	void printPairs(deque<pair> *p);
	void printMeasure(deque<measurement> *vec);
	/**
	* s.o.
	*/
	void printMeasure(struct measurement *m);
	/**
	* s.o.
	*/
	void printMeasure(struct measure *m);
	/**
	* Diese Methode gruppiert eine deque mit mehreren Messungen zu einer Mittleren Position
	* Kovarianzmatrizen werden berechnet und die Transformationen zur Mittleren Position
	*/
	void GroupSide(deque<measurement> *m);
	/**
	* Invertierung einer Messreihe
	*/
	void InvSide(deque<measurement> *m);
	/**
	* Berechnung des Transformationsvektors zwischen *pos_1 und *pos_2
	*/
	void calcTransformVec(double *pos_1, double *pos_2, double *newPos);
	/**
	* Matrix Ausgabe
	*/
	void printMatrix(math::Mat<3,3,double> *m);
	void printMatrix(math::Mat<2,2,double> *m);
	void printMatrix(math::Mat<3,2,double> *m);
	void printMatrix(math::Mat<2,3,double> *m);
	//s.o.
	void printMatrix(double *m);
	//Ausgabe der DatenStruktur
	void printDataStructure(deque<measurement> *d);
	//@depricated eigentliche Methode zum Bestimmen der Mittleren Messung
	//double calcMiddle(deque<measurement> *d);
	/**
	* sämtliche Koordinaten werden /10 gerechnet
	*/
	void divideTen(ArPose *robopos);
	void divideTen(double *m);
	void divideTen(math::Mat<3,3,double> *m);
	/**
	* sämtliche Koordinaten werden *10 gerechnet
	*/
	void mulTen(ArPose *robopos);
	void mulTen(double *m);
	/**
	* kopieren von Messungen
	*/
	void copyMeasures(struct measure *from, struct measure *to);
	/**
	* Diese Metode clont einen Scan und verschiebt diesen um var
	*/
	void cloneANDmove(std::deque<measurement> *scan_ref, std::deque<measurement> *scan_new, double x_var, double y_var, double rot);
	/**
	* Hier werden zwei Scans übergeben welche mittels getMahalanobis überprüft werden ob korrespondierende Punkte
	* vorhanden sind. Zurückgegeben wird eine Menge von Punkten die Korrespondieren
	*/
	void matchScans(measurement Scan0, measurement Scan1, deque<pair> *p);
	/**
	* Berechnung der MahalanobisDistanz zwischen zwei Punkten
	*/
	double getMahalanobis(double *xAB, double *covAB, measure *pi, measure *qi, struct pair *p);
	void groundTruth(deque<measurement> *groupedScan);
	int compare2Scans(deque<measurement> *Scan_0, deque<measurement> *Scan_1);
	void visualPairs(deque<pair> *set, img::ByteRGBImage *bildbla, img::ByteRGBImageDialog *dlg);
	//void visualPairs(deque<measurement> *set_0, deque<measurement> *set_1, img::ByteRGBImage *bild);
	void pseudoMove(deque<measurement> *m);
	void deletePixel(img::ByteRGBImage *b, img::ByteRGBImageDialog *d, int value, deque<measurement> *m);
	void deletePixel(img::ByteRGBImage *b, img::ByteRGBImageDialog *d);
	void filterPairs(deque<pair> *p);
	deque<measurement> RohScan_0;
	deque<measurement> RohScan_1;
	deque<measurement> GroupScan_0;
	deque<measurement> GroupScan_1;
};

#endif