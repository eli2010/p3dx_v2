#include "mapping.h"
//#include <list>
//working @Line 577 "matchScans"

mapping::mapping(ArRobot *p3dx) {
	this->p3dxptr = p3dx;
	this->SonarGlobalPose = new ArPose[anzsensor];
	this->SonarLocalPose = new ArPose[anzsensor];
	this->karte = new OccupancyGridMap(size, size);
	this->karte->setResolution(res);
	this->anzMessungen = 0;
	this->b.SetSize(size, size);
	this->bild.SetSize(size, size);
	for(int i = 0; i < bild.GetSizeX(); i++) {
		for(int j = 0; j<bild.GetSizeY(); j++) {
			bild.SetPixel(i, j, 255);
		}
	}
	this->paarebild.SetSize(size, size);
	for(int i = 0; i < paarebild.GetSizeX(); i++) {
		for(int j = 0; j<paarebild.GetSizeY(); j++) {
			paarebild.SetPixel(i, j, 255);
		}
	}
	this->demobild.SetSize(size, size);
	for(int i = 0; i < demobild.GetSizeX(); i++) {
		for(int j = 0; j<demobild.GetSizeY(); j++) {
			demobild.SetPixel(i, j, 255);
		}
	}
}
void mapping::calcSonar(int SensorID) {
	double range;
	double pos[2];
	double th;
	double robopos[3];
	ArPose currentPose = this->p3dxptr->getPose();
	robopos[0] = currentPose.getY()/res+offset_y;
	robopos[1] = currentPose.getX()/res+offset_x;
	robopos[2] = currentPose.getTh();
	this->karte->updateRobotPosition(robopos[0], robopos[1], this->p3dxptr->getRobotRadius()/res);
	range = (double) this->p3dxptr->getSonarRange(SensorID)/res;
	this->SonarGlobalPose[SensorID].getPose(&pos[1], &pos[0], &th);
	pos[0] = pos[0]/res+offset_y;
	pos[1] = pos[1]/res+offset_x;
	if(range <= (OBSTACLE_RANGE/res)) {
		for(double i = -15; i <= 15; i += 0.25) {
			this->karte->updateMap(pos, th+i+270, range);
		}
	}

	this->karte->checkMapExtension(pos, (double) offset_x, (double) offset_y);
	this->karte->convertToImage(b, robopos, true, p3dxptr->getRobotRadius()/res);
	this->dlg.ChangeImage(b);
}

void mapping::updateSonarLocalPose() {
	for(int i = 0; i < anzsensor; i++) {
		this->SonarLocalPose[i] = this->p3dxptr->getSonarReading(i)->getSensorPosition();
	}
}
void mapping::updateSonarGlobalPose() {
	ArPose currentPose = this->p3dxptr->getPose();
	ArTransform t;
	t.setTransform(currentPose);
	for(int j = 0; j < anzsensor; j++) {
		this->SonarGlobalPose[j] = t.doTransform(this->SonarLocalPose[j]);
	}
}
void mapping::startMapping() {
	while(p3dxptr->isConnected()) {
		this->updateSonarLocalPose();
		this->updateSonarGlobalPose();
		this->saveMeasure(p3dxptr, &messungen);
		for(int i = 0; i < anzsensor; i++) {
			this->calcSonar(i);
			//_getch();
		}
		Sleep(500);
	}
}
void mapping::ManuellMapping() {
	this->updateSonarLocalPose();
	this->updateSonarGlobalPose();
	this->saveMeasure(p3dxptr, &messungen);
	for(int i = 0; i < anzsensor; i++) {
		this->calcSonar(i);
	}
}
void *iniMapThread(LPVOID arg) {
	mapping *ptr = (mapping*)arg;
	ptr->startMapping();
	return 0;	
}
void mapping::runMapping() {
	int rc;
	for(int i = 0; i < NUM_THREADS; i++) {
		rc = pthread_create(&threads[i], NULL, iniMapThread, (void*)this);
		if (rc){
			printf("ERROR; return code from pthread_create() is %d\n", rc);
			exit(-1);
		}
	}
}
void mapping::saveMeasure(ArRobot *robo, deque<measurement> *m) {
	ArTransform t;
	ArPose currentPose = robo->getPose();
	//cout << "Aktuelle RoboPose der Messung: X: "<<currentPose.getX() << " Y: "<<currentPose.getY()<<" Th/Z: "<<currentPose.getTh()<<endl;;
	if(this->obstacleInRange()) {
		struct measurement neu;
		if(messungen.size() == 0) {
			t.setTransform(robo->getPose());
		} else if(messungen.size() > 0) {
			t.setTransform(messungen[messungen.size()-1].roboPose);
		}
		neu.roboPose.setPose(robo->getPose());
		neu.roboPose.setPose(t.doInvTransform(neu.roboPose));
		this->divideTen(&neu.roboPose);
		neu.robo_vec[0] = neu.roboPose.getX();
		neu.robo_vec[1] = neu.roboPose.getY();
		neu.robo_vec[2] = neu.roboPose.getTh();
		// initial wird hier auf Einheitsmatrix gesetzt
		for(int k = 0; k < 3; k++) {
			for(int j = 0; j < 3; j++) {
				if((k == j)) {
					neu.robo_cov[k*3+j] = 0.1;			
				} else
					neu.robo_cov[k*3+j] = 0;
			}
		}
		//ArTransform t2;
		//t2.setTransform(currentPose);
		for(int i = 0; i < anzsensor; i++) {
			if(robo->getSonarRange(i) < 3000) {
				anzMessungen++;
				double range = robo->getSonarRange(i);
				struct measure neu2;
				neu2.range[0] = robo->getSonarRange(i);
				neu2.range[1] = 0;
				neu2.range[2] = 0;
				ArPose SonarGlobalArPose;
				ArPose SonarLocalArPose;
				ArPose RangePose;
				SonarLocalArPose.setPose(robo->getSonarReading(i)->getSensorPosition().getX(), robo->getSonarReading(i)->getSensorPosition().getY(), robo->getSonarReading(i)->getSensorPosition().getTh());
				t.setTransform(robo->getPose());
				SonarGlobalArPose.setPose(t.doTransform(SonarLocalArPose));
				//SonarGlobalArPose.getPose(&neu2.SonarPoseGlobal[0],&neu2.SonarPoseGlobal[1],&neu2.SonarPoseGlobal[2]);
				RangePose.setPose(neu2.range[0],neu2.range[1],neu2.range[2]);
				t.setTransform(SonarGlobalArPose);
				RangePose.setPose(t.doTransform(RangePose));
				this->divideTen(&RangePose);
				RangePose.getPose(&neu2.range[0],&neu2.range[1],&neu2.range[2]);
				for(int j = 0; j < 3; j++) {
					for(int k = 0; k < 3; k++) {
						if((j == 0) && (k == 0)) {
							neu2.measure_cov[j*3+k] = (range/100)*(range/100);
						} else if((j == 1) && (k == 1)) {
							neu2.measure_cov[j*3+k] = (range/2)*tan(SONAR_ANGLE/2)*(range/2)*tan(SONAR_ANGLE/2);
						} else {
							neu2.measure_cov[j*3+k] = 0;
						}
					}
				}
				this->divideTen(neu2.measure_cov);
				neu.SonarMeasures.push_back(neu2);
			}
		}
		m->push_back(neu);
	}
}
void mapping::printMeasure(deque<measurement> *vec) {
	//cout << "gebe messuungen aus" << endl;
	cout << "Größe von messungen: " << vec->size() << endl;
	//cout << "Kovarianz Matrix der messungen" << endl;
	for(int i = 0; i < vec->size(); i++) {
		//cout << "Messungs Nr: " << messungen[i].number << endl;
		cout << "Position des Robos bei Messung: " << i << "; " << (*vec)[i].roboPose.getX() << " " << (*vec)[i].roboPose.getY() << " " << (*vec)[i].roboPose.getTh() << endl;
		cout << "kovarianz matrix" << endl;
		for(int k = 0; k < 3; k++) {
			for(int j = 0; j<3; j++) {
				cout << (*vec)[i].robo_cov[k*3+j] << " ";
			}
			cout << endl;
		}
		cout << "Messungen an der Stelle: "<< (*vec)[i].SonarMeasures.size()<<endl;
		for(int l = 0; l < (*vec)[i].SonarMeasures.size(); l++) {
			cout << (*vec)[i].SonarMeasures[l].range[0] << " "<< (*vec)[i].SonarMeasures[l].range[1]<< " " << (*vec)[i].SonarMeasures[l].range[2]<<endl;
		}
	}

}
bool mapping::obstacleInRange() {
	for(int i = 0; i < anzsensor; i++) {
		if(this->p3dxptr->getSonarRange(i) <= OBSTACLE_RANGE) {
			return true;
		}
	}
	return false;
}
/**
*	Diese Methode generiert Transformationen zwischen den Robo Positionen 0 und 1 2 und 3 usw...
*
*/
void mapping::groupMeasures() {
	this->groupMeasures(&messungen, &groupedMeasures);
}
void mapping::groupMeasures(deque<measurement> *orig, deque<measurement> *grouped) {
	deque<measurement> leftList;
	deque<measurement> rightList;
	double mitte = (orig->size()/2);
	//this->printMeasure(&messungen[mitte]);
	for(int i = 0; i <= mitte; i++) {
			leftList.push_back((*orig)[i]);
	}
	//this->visualMeasures(&leftList, &demobild, &demodlg, true);
	for(int i = mitte; i < orig->size(); i++) {
			rightList.push_back((*orig)[i]);
	}
	//this->visualMeasures(&rightList, &paarebild, &paardlg, false);
	//_getch();
	this->InvSide(&leftList);
	//this->visualMeasures(&leftList, &demobild, &demodlg, true);
	//_getch();
	while(leftList.size() > 1) {
		this->GroupSide(&leftList);
	}
	//this->visualMeasures(&leftList, &demobild, &demodlg, true);
	//_getch();
	while(rightList.size() > 1) {
		this->GroupSide(&rightList);
	}
	//this->visualMeasures(&rightList, &paarebild, &paardlg, false);
	//_getch();
	//nach diesem Schritt sollten nur noch 2 Transformationen von mitte nach aussen vorhanden sein.
	//diese Transformationen müssen nun wieder zu einem zusammen gefügt werden und mittels Grouping
	//Algorithmus zu einem zusammengefasst werden.
	grouped->push_back(leftList[0]);
	grouped->push_back(rightList[0]);
	this->GroupSide(grouped);
	//cout << "grouping done" << endl;
	//this->printMeasure(&groupedMeasures);
	orig->clear();
	leftList.clear();
	rightList.clear();
	//this->visualMeasures(grouped,&bild, &bilddlg, true);
	//this->visualMeasures(grouped,&demobild,&demodlg,true);
}

void mapping::calcCov(struct measurement *RoboPos_1, struct measurement *RoboPos_2, double *calcedCov) {
	double X2=(*RoboPos_2).roboPose.getX();
	double Y2=(*RoboPos_2).roboPose.getY();
	double THETA_1=(*RoboPos_1).roboPose.getThRad();
	double COS_THETA=cos(THETA_1);
	double SIN_THETA=sin(THETA_1);
	math::Mat<3,3,double> cov_1(1,0,0,0,1,0,((-X2*SIN_THETA)-(Y2*COS_THETA)), ((X2*COS_THETA)-(Y2*SIN_THETA)),1) ;
	math::Mat<3,3,double> cov_2(COS_THETA, SIN_THETA, 0, -SIN_THETA, COS_THETA,0,0,0,1);
	math::Mat<3,3,double> cov_1_T(1, 0, ((-X2*SIN_THETA)-(Y2*COS_THETA)), 0, 1, ((X2*COS_THETA)-(Y2*SIN_THETA)), 0, 0, 1);
	math::Mat<3,3,double> cov_2_T(COS_THETA, -SIN_THETA, 0,SIN_THETA, COS_THETA, 0,0,0,1);	
	math::Mat<3,3,double> RoboCov_1((*RoboPos_1).robo_cov[0], (*RoboPos_1).robo_cov[3],(*RoboPos_1).robo_cov[6], 
		(*RoboPos_1).robo_cov[1], (*RoboPos_1).robo_cov[4],(*RoboPos_1).robo_cov[7],
		(*RoboPos_1).robo_cov[2],(*RoboPos_1).robo_cov[5],(*RoboPos_1).robo_cov[8]);
	math::Mat<3,3,double> RoboCov_2((*RoboPos_2).robo_cov[0], (*RoboPos_2).robo_cov[3],(*RoboPos_2).robo_cov[6], 
		(*RoboPos_2).robo_cov[1], (*RoboPos_2).robo_cov[4],(*RoboPos_2).robo_cov[7],
		(*RoboPos_2).robo_cov[2],(*RoboPos_2).robo_cov[5],(*RoboPos_2).robo_cov[8]);
	math::Mat<3,3,double> ergebnis;
	ergebnis = (cov_1*RoboCov_1*cov_1_T)+(cov_2*RoboCov_2*cov_2_T);
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			calcedCov[i*3+j] = ergebnis[i][j];
		}
	}
}
void mapping::calcCov(struct measure *m, struct measurement *roboTrans, double *calcedCov) {
	double X2=(*roboTrans).roboPose.getX();
	double Y2=(*roboTrans).roboPose.getY();
	double THETA_1=(*m).range[2];
	double COS_THETA=cos(THETA_1);
	double SIN_THETA=sin(THETA_1);
	math::Mat<3,3,double> cov_1(1,0,0,0,1,0,((-X2*SIN_THETA)-(Y2*COS_THETA)), ((X2*COS_THETA)-(Y2*SIN_THETA)),1) ;
	math::Mat<3,3,double> cov_2(COS_THETA, SIN_THETA, 0, -SIN_THETA, COS_THETA,0,0,0,1);
	math::Mat<3,3,double> cov_1_T(1, 0, ((-X2*SIN_THETA)-(Y2*COS_THETA)), 0, 1, ((X2*COS_THETA)-(Y2*SIN_THETA)), 0, 0, 1);
	math::Mat<3,3,double> cov_2_T(COS_THETA, -SIN_THETA, 0,SIN_THETA, COS_THETA, 0,0,0,1);	
	math::Mat<3,3,double> mCov(m->measure_cov[0], m->measure_cov[3],m->measure_cov[6], 
		m->measure_cov[1], m->measure_cov[4],m->measure_cov[7],
		m->measure_cov[2],m->measure_cov[5],m->measure_cov[8]);
	math::Mat<3,3,double> RoboCov_2((*roboTrans).robo_cov[0], (*roboTrans).robo_cov[3],(*roboTrans).robo_cov[6], 
		(*roboTrans).robo_cov[1], (*roboTrans).robo_cov[4],(*roboTrans).robo_cov[7],
		(*roboTrans).robo_cov[2],(*roboTrans).robo_cov[5],(*roboTrans).robo_cov[8]);
	math::Mat<3,3,double> ergebnis;
	ergebnis = (cov_1*mCov*cov_1_T)+(cov_2*RoboCov_2*cov_2_T);
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			calcedCov[i*3+j] = ergebnis[i][j];
		}
	}


}
void mapping::matrixTrans(math::Mat<3,3,double> *m, math::Mat<3,3,double> *m_erg) {
	math::Mat<3,3,double> test_1((*m));
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			(*m_erg)[i][j] = (*m)[j][i];
		}
	}
}
void mapping::InvTrans(struct measurement *Robo_Vec, double *erg_cov, double *erg_vec) {
	double x,y,th;
	x=(*Robo_Vec).roboPose.getX();
	y=(*Robo_Vec).roboPose.getY();
	th=(*Robo_Vec).roboPose.getThRad();
	double newX = ((-1)*x*cos(th))-(y*sin(th));
	double newY = (x*sin(th))-(y*cos(th));
	double newth = (-1)*th;
	math::Mat<3,3,double> J(-cos(th), sin(th), 0, -sin(th), -cos(th), 0, (x*sin(th))-(y*cos(th)), (x*cos(th))+(y*sin(th)), -1); 
	math::Mat<3,3,double> RoboCov((*Robo_Vec).robo_cov[0], (*Robo_Vec).robo_cov[3],(*Robo_Vec).robo_cov[6], 
		(*Robo_Vec).robo_cov[1], (*Robo_Vec).robo_cov[4],(*Robo_Vec).robo_cov[7],
		(*Robo_Vec).robo_cov[2],(*Robo_Vec).robo_cov[5],(*Robo_Vec).robo_cov[8]);
	math::Mat<3,3,double> J_T(-cos(th), -sin(th), (x*sin(th))-(y*cos(th)), sin(th), -cos(th), (x*cos(th))+(y*sin(th)), 0, 0, -1);
	math::Mat<3,3,double> ergebnis;
	ergebnis = J*RoboCov*J_T;
	for(int i = 0; i < 3; i++) {
		for(int j=0; j < 3; j++) {
			erg_cov[i*3+j] = ergebnis[i][j];
		}
	}
	erg_vec[0] = newX;
	erg_vec[1] = newY;
	erg_vec[2] = newth;
}
void mapping::visualisieren() {
	this->visualMeasures(&messungen,&bild, &bilddlg, 1);
}
void mapping::visualPairs(deque<pair> *set, img::ByteRGBImage *bildbla, img::ByteRGBImageDialog *dlg) {
	//cout << "Rufe visualPairs auf" << endl;
	if(set->size() == 0) {
		cout << "Es wurden keine Paare gefunden" << endl;
		return;
	}
	img::ByteRGB blue(0, 0, 128);
	img::ByteRGB red(255,0,0);
	img::ByteRGB green(0,255,0);
	img::ByteRGB black(0, 0, 0);
	double pos_a[2];
	double pos_b[2];

	//cout << " Setze RoboPosen" << endl;

	//cout << "Robo gezeichnet" << endl;
	//cout << "MeasurePairs größe innerhalb der methode" << set->size() << endl;
	for(int i = 0; i < set->size(); i++) {
		pos_a[0] = (*set)[i].rob_pos_a[0]*10;
		pos_a[1] = (*set)[i].rob_pos_a[1]*10;
		pos_b[0] = (*set)[i].rob_pos_b[0]*10;
		pos_b[1] = (*set)[i].rob_pos_b[1]*10;
		img::DrawCircle((*bildbla), pos_a[1]/res+offset_y, pos_a[0]/res+offset_x, 4, blue, 2);
		img::DrawCircle((*bildbla), pos_b[1]/res+offset_y, pos_b[0]/res+offset_x, 4, black, 2);
		double p_0[2];
		double p_1[2];
		p_0[0] = (*set)[i].a_vec[0]*10;
		p_0[1] = (*set)[i].a_vec[1]*10;
		p_1[0] = (*set)[i].b_vec[0]*10;
		p_1[1] = (*set)[i].b_vec[1]*10;
		img::DrawCircle((*bildbla), p_0[1]/res+offset_y, p_0[0]/res+offset_y, 4, red, 2);
		img::DrawCircle((*bildbla), p_1[1]/res+offset_y, p_1[0]/res+offset_y, 4, green, 2);
		img::DrawLine((*bildbla), p_0[1]/res+offset_y, p_0[0]/res+offset_y, p_1[1]/res+offset_y, p_1[0]/res+offset_y, black, 1);
		//cout << "Punkt 1: " << p_0[1] << " " << p_0[0] << endl;
		//cout << "Punkt 2: " << p_1[1] << " " << p_1[0] << endl;
		//cout << "Maha distance der Punkte: " << (*set)[i].mahalaDis << endl;
		//_getch();
		
	}
	dlg->ChangeImage((*bildbla));
	//cout << "Beende visual pairs" << endl;
}
void mapping::visualRoboPose(measurement *measure, img::ByteRGBImage *b, img::ByteRGBImageDialog *d, int red) {
	img::ByteRGB blue(0, 0, 128);
	img::ByteRGB m;
	if(red == 1) {
		m.red = 255;
		m.green = 0;
		m.blue = 0;
	} else if(red == 2){
		m.red = 0;
		m.green = 255;
		m.blue = 0;
	} else if(red == 3) {
		m.red = 128;
		m.green = 128;
		m.blue = 128;
	}
	img::ByteRGB black(0, 0, 0);
	double pos[2];

	pos[0] = measure->roboPose.getX()*10;
	pos[1] = measure->roboPose.getY()*10;
	img::DrawCircle((*b), pos[1]/res+offset_y, pos[0]/res+offset_x, 4, black,2);
	d->ChangeImage((*b));
}
void mapping::visualMeasures(deque<measurement> *vec,img::ByteRGBImage *bild, img::ByteRGBImageDialog *dlg, int red) {
	//ArTransform t;
	img::ByteRGB blue(0, 0, 128);
	img::ByteRGB m;
	if(red == 1) {
		m.red = 255;
		m.green = 0;
		m.blue = 0;
	} else if(red == 2){
		m.red = 0;
		m.green = 255;
		m.blue = 0;
	} else if(red == 3) {
		m.red = 128;
		m.green = 128;
		m.blue = 128;
	}
	
	img::ByteRGB black(0, 0, 0);
	//bild.SetSize(size, size);
	double pos[2];
	//this->mulTen(&(*vec)[0].roboPose);
	//this->divideTen(&(*vec)[0].roboPose);
	//img::DrawCircle((*bild), pos[1]/res+offset_y, pos[0]/res+offset_x, 4, blue, 2);
	for(int i = 0; i < vec->size(); i++) {
		pos[0] = (*vec)[i].roboPose.getX()*10;
		pos[1] = (*vec)[i].roboPose.getY()*10;
		img::DrawCircle((*bild), pos[1]/res+offset_y, pos[0]/res+offset_x, 4, blue, 2);
		for(int j = 0; j < (*vec)[i].SonarMeasures.size(); j++) {
			//ArPose test;
			double RangePos[2];
			RangePos[0] = (*vec)[i].SonarMeasures[j].range[0]*10;
			RangePos[1] = (*vec)[i].SonarMeasures[j].range[1]*10;
			double x = RangePos[1]/res+offset_x;
			double y = RangePos[0]/res+offset_x;
			if(y > 1024 || x > 1024) {
				cout << "Werte größer als das Bild continue" << endl;
				continue;
			}
			//cout << x << " " << y<<endl;
			img::DrawCircle((*bild), x, y , 4, m, 2);
			//bilddlg.ChangeImage(bild);
			
		}
	}
	dlg->ChangeImage((*bild));
	////img::SaveBMP(bild, "gespeicherteMessungen.BMP");
	////img::ByteImage occu;
	//occu.SetSize(size, size);
	//pos[0] = pos[0]/res+offset_x;
	//pos[1] = pos[1]/res+offset_x;
	//this->karte->convertToImage(occu, pos);
	//this->dlg.ChangeImage(occu);
	//img::SaveBMP(occu, "Occu.BMP");
}

void mapping::printPairs(deque<pair> *p) {
	for(int i = 0; i < p->size(); i++) {
		cout << "A: " << (*p)[i].a_vec[0] << " " << (*p)[i].a_vec[1] << " " <<(*p)[i].a_vec[2] << endl;		
		cout << "B: " << (*p)[i].b_vec[0] << " " << (*p)[i].b_vec[1] << " " <<(*p)[i].b_vec[2] << endl;
		cout << "MahaDistanz für das Paar: " << (*p)[i].mahalaDis << endl;
	}
}
void mapping::printMeasure(mapping::measurement *m) {
	cout << "Vektor:" << m->roboPose.getX() << " " << m->roboPose.getY() << " " << m->roboPose.getTh() << endl;
	for(int i = 0; i<3; i++) {
		for(int j = 0; j<3; j++) {
			cout << m->robo_cov[i*3+j] << " ";
		}
		cout << endl;
	}
}
void mapping::printMeasure(mapping::measure *m) {
	cout << "kovarianz der Messung" << endl;
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			cout << (*m).measure_cov[i*3+j] << " ";
		}
		cout << endl;
	}
	cout << "range vektor der messung" <<endl;
	for(int i = 0; i < 3; i++) {
		cout << (*m).range[i] << " ";
	}
	cout << endl;
}
void mapping::GroupSide(std::deque<measurement> *v) {
	double tempVec_1[3];
	double tempVec_2[3];
	(*v)[0].roboPose.getPose(&tempVec_1[0], &tempVec_1[1], &tempVec_1[2]);
	(*v)[1].roboPose.getPose(&tempVec_2[0], &tempVec_2[1], &tempVec_2[2]);
	tempVec_1[2] = (*v)[0].roboPose.getTh();
	tempVec_2[2] = (*v)[1].roboPose.getTh();
	struct measurement neu;
	this->calcTransformVec(tempVec_1, tempVec_2, neu.robo_vec); 
	neu.roboPose.setPose(neu.robo_vec[0], neu.robo_vec[1], neu.robo_vec[2]); 
	this->calcCov(&(*v)[0], &(*v)[1], neu.robo_cov);
//	this->calcInvVec(neu.robo_vec, neu.robo_vec_Inv);
	/* kopieren der Messungen vom Objekt 0 */
	for(int n = 0; n < (*v)[0].SonarMeasures.size(); n++) {
		struct measure neu2;
		//struct measure temp;
		//this->copyMeasures(&(*v)[0].SonarMeasures[n], &temp);
		this->calcTransformVec((*v)[0].SonarMeasures[n].range, neu.robo_vec, neu2.range);
		this->calcCov(&(*v)[0].SonarMeasures[n], &neu, neu2.measure_cov);
//		this->calcTransformVec((*v)[0].SonarMeasures[n].SonarPoseGlobal, neu.robo_vec, neu2.SonarPoseGlobal);
//		this->calcTransformVec(neu.robo_vec_Inv, (*v)[0].SonarMeasures[n].range, neu2.range);
		neu.SonarMeasures.push_back(neu2);
		//this->printMeasure(&(*v)[0].SonarMeasures[n]);
		//this->printMeasure(&neu2);
		//_getch();
	}
	/* kopieren der Messungen von Objekt 1 */
	for(int n = 0; n < (*v)[1].SonarMeasures.size(); n++) {
		struct measure neu2;
		this->copyMeasures(&(*v)[1].SonarMeasures[n], &neu2);
		neu.SonarMeasures.push_back(neu2);
	}
	(*v).pop_front();
	(*v).pop_front();
	(*v).push_front(neu);
}
void mapping::InvSide(std::deque<measurement> *v) {
	deque<measurement> tmp;
	for(int i = 0; i < (*v).size(); i++) {
		struct measurement neu;
		this->InvTrans(&(*v)[i], neu.robo_cov, neu.robo_vec);
		neu.roboPose.setPose(neu.robo_vec[0], neu.robo_vec[1], neu.robo_vec[2]);
		for(int n = 0; n < (*v)[i].SonarMeasures.size(); n++) {
			struct measure neu2;
			for(int h = 0; h < 3; h++) {
				neu2.range[h] = (*v)[i].SonarMeasures[n].range[h];
//				neu2.SonarPoseLocal[h] = (*v)[i].SonarMeasures[n].SonarPoseLocal[h];
//				neu2.SonarPoseGlobal[h] = (*v)[i].SonarMeasures[n].SonarPoseGlobal[h];
			}
			for(int p = 0; p < 3; p++) {
				for(int o = 0; o<3;o++) {
					neu2.measure_cov[p*3+o] = (*v)[i].SonarMeasures[n].measure_cov[p*3+o];
				}
			}
			neu.SonarMeasures.push_back(neu2);
		}
		tmp.push_back(neu);
	}
	tmp.swap((*v));
}
void mapping::testMethods() {
	//struct measurement roboStart;
	//roboStart.robo_vec[0] = 0;
	//roboStart.robo_vec[1] = 0;
	//roboStart.robo_vec[2] = 0;
	//roboStart.roboPose.setPose(roboStart.robo_vec[0],roboStart.robo_vec[1],roboStart.robo_vec[2]);
	//for(int k = 0; k < 3; k++) {
	//	for(int j = 0; j < 3; j++) {
	//		if((k == j)) {
	//			roboStart.robo_cov[k*3+j] = 0.1;			
	//		} else
	//			roboStart.robo_cov[k*3+j] = 0;
	//	}
	//}
	//messungen.push_back(roboStart);
	//for(int i = 0; i < 5; i++) {
	//	ArTransform t;
	//	t.setTransform(messungen[messungen.size()-1].roboPose);
	//	struct measurement neu;
	//	neu.robo_vec[0] = roboStart.robo_vec[0]+50;
	//	neu.robo_vec[1] = roboStart.robo_vec[1];
	//	neu.robo_vec[2] = roboStart.robo_vec[2];
	//	neu.roboPose.setPose(neu.robo_vec[0],neu.robo_vec[1],neu.robo_vec[2]);
	//	neu.roboPose.setPose(t.doInvTransform(neu.roboPose));
	//	this->divideTen(&neu.roboPose);
	//	neu.roboPose.getPose(&neu.robo_vec[0],&neu.robo_vec[0],&neu.robo_vec[0]);
	//	for(int k = 0; k < 3; k++) {
	//		for(int j = 0; j < 3; j++) {
	//			if((k == j)) {
	//				neu.robo_cov[k*3+j] = 0.1;			
	//			} else
	//				neu.robo_cov[k*3+j] = 0;
	//		}
	//	}
	//	for(int j = 0; j < anzsensor; j++) {
	//		struct measure neu2;
	//		int r = rand();
	//		//cout << r << endl;
	//		//_getch();
	//		while((r > 5000) || (r < 500)) {
	//			r = rand();
	//			//cout << "Randomzahl eingefügt: " << r<<endl;
	//			//_getch();
	//		}
	//		neu2.range[0] = r;
	//		neu2.range[1] = 0;
	//		neu2.range[2] = 0;
	//		for(int j = 0; j < 3; j++) {
	//			for(int k = 0; k < 3; k++) {
	//				if((j == 0) && (k == 0)) {
	//					neu2.measure_cov[j*3+k] = (r/100)*(r/100);
	//				} else if((j == 1) && (k == 1)) {
	//					neu2.measure_cov[j*3+k] = (r/2)*tan(SONAR_ANGLE/2)*(r/2)*tan(SONAR_ANGLE/2);
	//				} else {
	//					neu2.measure_cov[j*3+k] = 0;
	//				}
	//			}
	//		}
	//		this->divideTen(neu2.measure_cov);
	//		ArTransform t2;
	//		t2.setTransform(neu.roboPose);
	//		ArPose rangepose;
	//		rangepose.setPose(neu2.range[0],neu2.range[1],neu2.range[2]);
	//		rangepose.setPose(t2.doTransform(rangepose));
	//		this->divideTen(&rangepose);
	//		rangepose.getPose(&neu2.range[0],&neu2.range[1],&neu2.range[2]);
	//		neu.SonarMeasures.push_back(neu2);
	//	}
	//	this->messungen.push_back(neu);
	//}
	//this->visualMeasures(&messungen, &bild, &bilddlg,true);
	//this->printMeasure(&messungen);
	//this->groupMeasures();
	/*math::Mat<3,3,double> ergebnis;
	math::Mat<3,3,double> temp;
	math::Mat<3,3,double> matrix_1(1,2,3,4,5,6,7,8,9);
	math::Mat<3,3,double> matrix_2(1,4,7,2,5,8,3,6,9);
	this->printMatrix(&matrix_1);
	//this->matrixTrans(&matrix_2, &temp);
	//this->matrixMul(matrix_1, temp, ergebnis);
	this->printMatrix(&matrix_2);
	//this->printMatrix(&temp);
	ergebnis = matrix_2*matrix_1;
	this->printMatrix(&ergebnis);
	//ergebnis.Inv();
	//printMatrix(&ergebnis);
	//double temp_2[9];
	//this->matrixAdd(matrix_1, temp, temp_2);
	//this->printMatrix(temp_2);
	//this->calcCov(
	*/
	//math::Mat<2,2,double> t(1,3,2,4);
	//math::Vec2d v(1,1);
	//math::Mat<2,2,double> erg;
	//cout << v[0] << " " << v[1] << endl;
	//cout << t[0][0] << " " << t[0][1] <<  " " << t[1][0] << " " <<t[1][1] << endl;
	//cout << (v*t).GetDimm()<< " " << (v*t).GetDimn() << endl;
	//cout << erg[0] << " " << erg[1];
	//getMahalanobis umschreiben sodass 3x2 Matrix benutzt wird
	math::Mat<3,2,double> test;
	test[0][0] = 1;
	test[0][1] = 0;
	test[0][2] = 0;
	test[1][0] = 0;
	test[1][1] = 1;
	test[1][2] = 1;
	math::Mat<2,2,double> test2(1,0,0,1);
	
	//test[0][2] = 1;
	//test[1][2] = 0;
	//cout << "test1"<<endl;
	//cout << test[0][0] << " " <<test[0][1] << " " << test[0][2] << endl;
	//cout << test[1][0] << " " <<test[1][1] << " " << test[1][2] << endl;
	//this->printMatrix(&test);
	cout << "test2"<<endl;
	cout << test2[0][0] << " " << test2[0][1] << endl;
	cout << test2[1][0] << " " << test2[1][1] << endl;
	this->printMatrix(&test2);
}
void mapping::printMatrix(math::Mat<3,3,double> *m) {
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			cout << (*m)[j][i] << " ";
		}
		cout << endl;
	}
}
void mapping::printMatrix(math::Mat<2,3,double> *m) {
	for(int i = 0; i < 2; i++) {
		for(int j = 0; j < 3; j++) {
			cout << (*m)[j][i] << " ";
		}
		cout << endl;
	}
}
void mapping::printMatrix(math::Mat<3,2,double> *m) {
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 2; j++) {
			cout << (*m)[j][i] << " ";
		}
		cout << endl;
	}
}
void mapping::printMatrix(math::Mat<2,2,double> *m) {
	for(int i = 0; i < 2; i++) {
		for(int j = 0; j < 2; j++) {
			cout << (*m)[j][i] << " ";
		}
		cout << endl;
	}
}
void mapping::printMatrix(double *m) {
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j <3; j++) {
			cout << m[j*3+i] << " ";
		}
		cout << endl;
	}
}
void mapping::calcTransformVec(double *pos_1, double *pos_2, double *newPos) {
	newPos[0] = (pos_1[0])+(pos_2[0]*cos(pos_1[2]*(M_PI/180)))-(pos_2[1]*sin(pos_1[2]*(M_PI/180))); 
	newPos[1] = (pos_1[1])+(pos_2[0]*sin(pos_1[2]*(M_PI/180)))+(pos_2[1]*cos(pos_1[2]*(M_PI/180)));
	newPos[2] = (pos_1[2]+pos_2[2]);
}
void mapping::calcInvVec(double *v) {
	v[0] = ((-1)*v[0]*cos(v[2]*(M_PI/180)))-(v[1]*sin(v[2]*(M_PI/180)));
	v[1] = (v[0]*sin(v[2]*(M_PI/180)))-(v[1]*cos(v[2]*(M_PI/180)));
	v[2] = (-1)*v[2];
}
void mapping::calcInvVec(double *v, double *v_erg) {
	v_erg[0] = ((-1)*v[0]*cos(v[2]*(M_PI/180)))-(v[1]*sin(v[2]*(M_PI/180)));
	v_erg[1] = (v[0]*sin(v[2]*(M_PI/180)))-(v[1]*cos(v[2]*(M_PI/180)));
	v_erg[2] = (-1)*v[2];
}
void mapping::printDataStructure(deque<measurement> *d) {
	for(int i = 0; i < (*d).size(); i++) {
		cout << "Robo vec" << endl;
		for(int j = 0; j < 3; j++) {
			cout << (*d)[i].robo_vec[j] << " ";
		}
		cout<<endl;
		cout << "Robo Kov: " << i << endl;
		for(int k = 0; k < 3; k++) {
			for(int j = 0; j < 3; j++) {
				cout << (*d)[i].robo_cov[k*3+j] << " ";
			}	
			cout << endl;
		}
	}
}
//double mapping::calcMiddle(std::deque<measurement> *d) {
//	double sum = 0; 
//	for(int i = 0; i < (*d).size(); i++) {
////		sum +=(*d)[i].distance;
//	}
//	sum /= 2;
//	return sum;
//}
void mapping::divideTen(ArPose *robopos) {
	robopos->setPose(robopos->getX()/10, robopos->getY()/10, robopos->getTh());
}
void mapping::divideTen(double *m) {
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			m[i*3+j] /= 10;
		}
	}
}
void mapping::mulTen(ArPose *robopos) {
	robopos->setPose(robopos->getX()*10, robopos->getY()*10, robopos->getTh());
}
void mapping::mulTen(double *m) {
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			m[i*3+j] *= 10;
		}
	}
}
void mapping::copyMeasures(mapping::measure *from, mapping::measure *to) {
	/**
	* kopieren von messungen von *from zu *to
	*/
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j < 3; j++) {
			(*to).measure_cov[i*3+j] = (*from).measure_cov[i*3+j];
		}
	}
	for(int i = 0; i < 3; i++) {
		(*to).range[i] = (*from).range[i];
	}
}
void mapping::cloneANDmove(std::deque<measurement> *scan_ref, std::deque<measurement> *scan_new, double x_var, double y_var, double rot) {
	double x_off = x_var/10;
	double y_off = y_var/10;
	for(int i = 0; i < scan_ref->size(); i++) {
		struct measurement neu;
		neu.roboPose.setPose((*scan_ref)[i].roboPose);
		//this->mulTen(&neu.roboPose);
		neu.roboPose.getPose(&neu.robo_vec[0], &neu.robo_vec[1], (&neu.robo_vec[2]));
		//neu.roboPose.setPose(neu.robo_vec[0],neu.robo_vec[1],neu.robo_vec[2]);
		for(int k = 0; k < 3; k++) {
			for(int j = 0; j < 3; j++) {
				neu.robo_cov[k*3+j] = (*scan_ref)[i].robo_cov[k*3+j];
			}
		}
	//	cout << "Ref RoboPose: " << (*scan_ref)[i].roboPose.getX() << " " << (*scan_ref)[i].roboPose.getY() << " " << (*scan_ref)[i].roboPose.getTh() << endl; 
		neu.robo_vec[0] += x_off;
		neu.robo_vec[1] += y_off;
		
		//neu.robo_vec[2] += rot;
		neu.roboPose.setPose(neu.robo_vec[0],neu.robo_vec[1],neu.robo_vec[2]);
		//cout << "New RoboPose: " << neu.roboPose.getX() << " " << neu.roboPose.getY() << " " << neu.roboPose.getTh() << endl; 
		//ArTransform t;
		//t.setTransform(neu.roboPose);
		for(int j = 0; j < (*scan_ref)[i].SonarMeasures.size(); j++) {
			//Notiz: Ausgabe der Messungen um zuüberprüfen ob diese Richtig verschoben werden!
			struct measure neu2;
		//	cout << "Ref Messung: " << (*scan_ref)[i].SonarMeasures[j].range[0] <<" "<< (*scan_ref)[i].SonarMeasures[j].range[1] << " " << (*scan_ref)[i].SonarMeasures[j].range[2] << endl;
			/*ArPose transPose;
			//ArPose fakePose;
			fakePose.setPose((neu.roboPose.getX()+(x_var/2)),(neu.roboPose.getY()+(y_var/2)),neu.roboPose.getTh());
			t.setTransform(fakePose);
			transPose.setPose((*scan_ref)[i].SonarMeasures[j].range[0],(*scan_ref)[i].SonarMeasures[j].range[1],(*scan_ref)[i].SonarMeasures[j].range[2]);
			transPose.setPose(t.doTransform(transPose));
			//this->divideTen(&transPose);
			transPose.getPose(&neu2.range[0],&neu2.range[1],&neu2.range[2]);
			cout << "New Messung: " << neu2.range[0] << " " << neu2.range[1] << " " << neu2.range[2] << endl;
			cout << "--------------------"<<endl;
			if(rot != 0) {
				double x = neu2.range[0];
				double y = neu2.range[1];
				neu2.range[0] = x*cos(rot*(M_PI/180))-y*sin(rot*(M_PI/180));
				neu2.range[1] = y*cos(rot*(M_PI/180))+x*sin(rot*(M_PI/180));
			}*/
			neu2.range[0] = (*scan_ref)[i].SonarMeasures[j].range[0]+(x_off/2);
			neu2.range[1] = (*scan_ref)[i].SonarMeasures[j].range[1]+(y_off/2);
			neu2.range[2] = (*scan_ref)[i].SonarMeasures[j].range[2];
			cout << "New Messung: " << neu2.range[0] << " " << neu2.range[1] << " " << neu2.range[2] << endl;
			for(int k = 0; k < 3; k++) {
				for(int l = 0; l < 3; l++) {
					neu2.measure_cov[k*3+l] = (*scan_ref)[i].SonarMeasures[j].measure_cov[k*3+l];
				}
			}
			/*for(int j = 0; j < 3; j++) {
				for(int k = 0; k < 3; k++) {
					if((j == 0) && (k == 0)) {
						neu2.measure_cov[j*3+k] = (range/100)*(range/100);
					} else if((j == 1) && (k == 1)) {
						neu2.measure_cov[j*3+k] = (range/2)*tan(SONAR_ANGLE/2)*(range/2)*tan(SONAR_ANGLE/2);
					} else {
						neu2.measure_cov[j*3+k] = 0;
					}
				}
			}*/
			//neu2.range[0] += var;
			//neu2.range[1] += var;
			neu.SonarMeasures.push_back(neu2);	
		}
		//this->divideTen(&neu.roboPose);
		
		scan_new->push_back(neu);
	}
}
void mapping::matchScans(measurement Scan0, measurement Scan1, deque<pair> *p) {
	//paare mit gewissen distanzen aussortieren!
	//p->clear();
	double ScanVec01[3];
	double ScanCov01[9];
	this->calcTransformVec(Scan0.robo_vec, Scan1.robo_vec, ScanVec01);
	this->calcCov(&Scan0, &Scan1, ScanCov01);
	for(int i = 0; i < Scan0.SonarMeasures.size(); i++) {
		struct pair paar;
		double mydistance = 0;
		Scan0.roboPose.getPose(&paar.rob_pos_a[0],&paar.rob_pos_a[1],&paar.rob_pos_a[2]);
		Scan1.roboPose.getPose(&paar.rob_pos_b[0],&paar.rob_pos_b[1],&paar.rob_pos_b[2]);
		paar.a_vec[0] = Scan0.SonarMeasures[i].range[0];
		paar.a_vec[1] = Scan0.SonarMeasures[i].range[1];
		paar.a_vec[2] = Scan0.SonarMeasures[i].range[2];
	
		paar.mahalaDis = 99999999;
		for(int j = 0; j < Scan1.SonarMeasures.size(); j++) {
		//	cout << "getMaha Data: px: " << Scan0.SonarMeasures[i].range[0] <<" py: " <<Scan0.SonarMeasures[i].range[2] << endl;
		//	cout << "getMaha Data: px: " << Scan0.SonarMeasures[i].range[0] <<" py: " <<Scan0.SonarMeasures[i].range[2] << endl;
			mydistance = this->getMahalanobis(ScanVec01, ScanCov01, &Scan0.SonarMeasures[i], &Scan1.SonarMeasures[j], &paar);
			//hier muss ein minCount rein
			//disCounter += distance;
			
			if(mydistance < paar.mahalaDis) {
				paar.mahalaDis = mydistance;
				//cout << "Berechnete Dis: " << mydistance << endl;
				//paar.a_vec[0] = (*groupedScan)[0].SonarMeasures[i].range[0];
				//paar.a_vec[1] = (*groupedScan)[0].SonarMeasures[i].range[1];
				//paar.a_vec[2] = (*groupedScan)[0].SonarMeasures[i].range[2]; 
				paar.b_vec[0] = Scan1.SonarMeasures[j].range[0];
				paar.b_vec[1] = Scan1.SonarMeasures[j].range[1];
				paar.b_vec[2] = Scan1.SonarMeasures[j].range[2];
			}
		
		}
		p->push_back(paar);		
	}
	cout << "Groesse der (vor Filter) Paarmenge: "<<measurePairs.size()<< endl;
	this->filterPairs(&measurePairs);
	//this->printPairs(&measurePairs);
	//_getch();
	cout << "Groesse der (Nach Filter) Paarmenge: "<<measurePairs.size()<< endl;
	//this->deletePixel(&paarebild, &paardlg);
	//this->visualPairs(&measurePairs, &paarebild,&paardlg);
}
void mapping::filterPairs(std::deque<pair> *p) {
	deque<pair> temp;
	for(int i = 0; i < p->size(); i++) {
		if((((*p)[i].mahalaDis)*((*p)[i].mahalaDis)) < 0.0001) {
			temp.push_back((*p)[i]); 
		}
	}
	p->swap(temp);
}
double mapping::getMahalanobis(double *xAB, double *covAB, measure *pi, measure *qi, struct pair *p) {
	/**
	* Berechnungen der Mahalanobisdistanz zwischen 2 Messungen mittels dem transformation
	* Vektor zwischen 2 Scans und der dazugehörigen Kovarianzmatrix
	*/
	//Rad?
	double px = (*pi).range[0];
	double py = (*pi).range[1];
	double qx = (*qi).range[0];
	double qy = (*qi).range[1];
	double COS_X = cos(xAB[2]*(M_PI/180));
	double SIN_X = sin(xAB[2]*(M_PI/180));
	//math::Mat<2,3,double> test;
	//math::Mat<2,2,double> J_3(1,0,0,1);
	/*	J3
		1 0 ((-1)*px*SIN_X-py*COS_X)
		0 1 (px*COS_X-py*SIN_X)

		J3^T
		1							0
		0							1
 ((-1)*px*SIN_X-py*COS_X) (px*COS_X-py*SIN_X)
	*/
	//cout << "SIN_X: " << SIN_X << " COS_X: " << COS_X << endl;
	//cout << "px: " << px << " py: " << py << " (((-1)*px)*SIN_X-py*COS_X): " <<(((-1)*px)*SIN_X-py*COS_X) << " (px*COS_X-py*SIN_X): "<<(px*COS_X-py*SIN_X)<< endl;
	math::Mat<2,3,double> J_3;
	J_3[0][0] = 1; J_3[1][0] = 0; J_3[2][0] = (((-1)*px)*SIN_X-py*COS_X);
	J_3[0][1] = 0; J_3[1][1] = 1; J_3[2][1] = (px*COS_X-py*SIN_X);
	//cout << "J_3" << endl;
	//this->printMatrix(&J_3);
	math::Mat<2,2,double> J_4(COS_X, SIN_X, (-1)*SIN_X, COS_X);
	//cout << "J_4" << endl;
	//this->printMatrix(&J_4);
	math::Mat<3,2,double> J_3_T;
	J_3_T[0][0] = 1; J_3_T[1][0] = 0;
	J_3_T[0][1] = 0; J_3_T[1][1] = 1;
	J_3_T[0][2] = ((-1)*px*SIN_X-py*COS_X); J_3_T[1][2] = (px*COS_X-py*SIN_X);
	//cout << "J_3_T" << endl;
	//this->printMatrix(&J_3_T);
	//_getch();
	math::Mat<2,2,double> J_4_T(COS_X, (-1)*SIN_X, SIN_X, COS_X);
	//cout << "J_4_T" << endl;
	//this->printMatrix(&J_4_T);
	math::Mat<3,3,double> PAB(covAB[0], covAB[3],covAB[6], covAB[1], covAB[4],covAB[7],covAB[2],covAB[5],covAB[8]);
	//cout << "PAB" << endl;
	//this->printMatrix(&PAB);
	math::Mat<2,2,double> Ppi((*pi).measure_cov[0], (*pi).measure_cov[3], (*pi).measure_cov[1], (*pi).measure_cov[4]);
	//cout << "Ppi" << endl;
	//this->printMatrix(&Ppi);
	math::Mat<2,2,double> Pqi((*qi).measure_cov[0], (*qi).measure_cov[3], (*qi).measure_cov[1], (*qi).measure_cov[4]);
	//cout << "Pqi" << endl;
	//this->printMatrix(&Pqi);
	math::Mat<2,2,double> C;
	C = ((J_3*PAB)*J_3_T)+((J_4*Ppi)*J_4_T)+Pqi;
	p->C = C;
	//cout << "C" << endl;
	//this->printMatrix(&C);

	math::Vec<2,double> h((xAB[0]+px*COS_X-py*SIN_X)-qx, (xAB[1]+px*SIN_X+py*COS_X)-qy);
	//cout << "h" << h[0] << " " << h[1] << endl;
	
	C.Inv();
	//cout << "C.Inv" << endl;
	//this->printMatrix(&C);
	//_getch();
	p->J = J_3;
	p->J_T = J_3_T;
	p->h = h;
	p->C_1 = C;
	//return (h[0]*h[0]+h[1]*h[1]);
	return (h[0]*(h[0]*C[0][0]+h[1]*C[0][1]))+(h[1]*(h[0]*C[1][0]+h[1]*C[1][1]));

}
void mapping::groundTruth(deque<measurement> *groupedScan) {
	deque<measurement> clonedScan;
	this->cloneANDmove(groupedScan, &clonedScan, 50, 50, 0);
	this->visualMeasures(groupedScan, &bild,&bilddlg,1);
	this->visualMeasures(&clonedScan,&bild,&bilddlg, 2);
	this->matchScans((*groupedScan)[0], clonedScan[0], &measurePairs);
}
int mapping::compare2Scans(deque<measurement> *Scan_0, deque<measurement> *Scan_1) {
	int counter = 0;
	for(int i = 0; i < Scan_0->size(); i++) {
		for(int j = 0; j < Scan_1->size(); j++) {
			for(int k = 0; k < 3; k++) {
				for(int l = 0; l < 3; l++) {
					if((*Scan_0)[i].robo_cov[k*3+l] != (*Scan_1)[j].robo_cov[k*3+l]) {
						counter++;
					}
				}
			}
			for(int k = 0; k < 3; k++) {
				if((*Scan_0)[i].robo_vec[k] != (*Scan_1)[j].robo_vec[k]) {
					counter++;
				}
			}
			for(int k = 0; (k < (*Scan_0)[i].SonarMeasures.size()) && (k < (*Scan_1)[i].SonarMeasures.size()); k++) {
					for(int o = 0; o < 3; o++) {
						for(int p = 0; p < 3; p++) {
							if((*Scan_0)[i].SonarMeasures[k].measure_cov[o*3+p] != (*Scan_1)[j].SonarMeasures[k].measure_cov[o*3+p]) {
								counter++;
							}
						}
					}
					for(int o = 0; o < 3; o++) {
						if((*Scan_0)[i].SonarMeasures[k].range[o] != (*Scan_0)[j].SonarMeasures[k].range[o]) {
							counter++;
						}
					}
			}
		}
	}
	return counter;
}
void mapping::matching() {
	this->groundTruth(&groupedMeasures);
	//this->printDataStructure(&groupedMeasures);
}
void mapping::pseudoMove() {
	this->pseudoMove(&groupedMeasures);
}
void mapping::pseudoMove(deque<measurement> *m) {
	cout << "Starte DemoBewegungen" << endl;
	cout << "t initiale Testwerte generieren" << endl;
	cout << "g Test Werte gruppieren" << endl;
	cout << "b paare Berechnen" << endl;
	cout << "wasd steuern" <<endl;
	bool demovar = true;
	double w=0;
	double a=0;
	double rotate =0;
	//double s=0;
	//double d=0;
	//this->visualMeasures(m,&demobild,&demodlg,true);
	char key;
	deque<measurement> scan_2;
	//this->cloneANDmove(m, &scan_2,0,0,0);
	while(demovar) {
		if(kbhit()) {
			key = _getch();
			switch(key) {
				case 'w':
					w +=200;
					cout << "Verschiebung um: "<<" X: "<< w<<" Y: " << a << " Rot: " << rotate << endl;
					scan_2.clear();
					measurePairs.clear();
					this->cloneANDmove(m,&scan_2,w,a,rotate);
					this->matchScans((*m)[0],scan_2[0],&measurePairs);
					cout << "MeasurePairs Size: " << this->measurePairs.size() << endl;
					this->deletePixel(&demobild, &demodlg,0,m);
					this->visualMeasures(&scan_2, &demobild, &demodlg,3);
					this->visualPairs(&measurePairs, &demobild, &demodlg);
					this->visualRoboPose(&scan_2[0],&demobild,&demodlg,2);
					/*paare visualisieren! */
					img::SaveBMP(demobild,"verschobeneScans.BMP");
					//this->visua
					break;
				case 'a':
					a -=200;
					cout << "Verschiebung um: "<<" X: "<< w<<" Y: " << a << " Rot: " << rotate << endl;
					scan_2.clear();
					measurePairs.clear();
					this->cloneANDmove(m,&scan_2,w,a,rotate);
					this->matchScans((*m)[0],scan_2[0],&measurePairs);
					cout << "MeasurePairs Size: " << this->measurePairs.size() << endl;
					this->deletePixel(&demobild, &demodlg,0,m);
					this->visualMeasures(&scan_2, &demobild, &demodlg,3);
					this->visualRoboPose(&scan_2[0],&demobild,&demodlg,2);
					this->visualPairs(&measurePairs, &demobild, &demodlg);
					img::SaveBMP(demobild,"verschobeneScans.BMP");
					break;
				case 'd':
					a +=200;
					cout << "Verschiebung um: "<<" X: "<< w<<" Y: " << a << " Rot: " << rotate << endl;
					scan_2.clear();
					measurePairs.clear();
					this->cloneANDmove(m,&scan_2,w,a,rotate);
					this->matchScans((*m)[0],scan_2[0],&measurePairs);
					cout << "MeasurePairs Size: " << this->measurePairs.size() << endl;
					this->deletePixel(&demobild, &demodlg,0,m);
					this->visualMeasures(&scan_2, &demobild, &demodlg,3);
					this->visualRoboPose(&scan_2[0],&demobild,&demodlg,2);
					this->visualPairs(&measurePairs, &demobild, &demodlg);
					img::SaveBMP(demobild,"verschobeneScans.BMP");
					break;
				case 's':
					w -=200;
					cout << "Verschiebung um: "<<" X: "<< w<<" Y: " << a << " Rot: " << rotate << endl;
					scan_2.clear();
					measurePairs.clear();
					this->cloneANDmove(m,&scan_2,w,a,rotate);
					this->matchScans((*m)[0],scan_2[0],&measurePairs);
					cout << "MeasurePairs Size: " << this->measurePairs.size() << endl;
					this->deletePixel(&demobild, &demodlg,0,m);
					this->visualMeasures(&scan_2, &demobild, &demodlg,3);
					this->visualRoboPose(&scan_2[0],&demobild,&demodlg,2);
					this->visualPairs(&measurePairs, &demobild, &demodlg);
					img::SaveBMP(demobild,"verschobeneScans.BMP");
					break;
				/*case 'q':
					rotate -=5;
					cout << "Verschiebung um: "<<" X: "<< w<<" Y: " << a << " Rot: " << rotate << endl;
					scan_2.clear();
					measurePairs.clear();
					this->cloneANDmove(m,&scan_2,w,a,rotate);
					this->matchScans((*m)[0],scan_2[0],&measurePairs);
					cout << "MeasurePairs Size: " << this->measurePairs.size() << endl;
					this->deletePixel(&demobild, &demodlg,0,m);
					this->visualMeasures(&scan_2, &demobild, &demodlg,3);
					this->visualRoboPose(&scan_2[0],&demobild,&demodlg,1);
					this->visualPairs(&measurePairs, &demobild, &demodlg);
					img::SaveBMP(demobild,"verschobeneScans.BMP");
					break;
				case 'e':
					rotate +=5;
					cout << "Verschiebung um: "<<" X: "<< w<<" Y: " << a << " Rot: " << rotate << endl;
					scan_2.clear();
					measurePairs.clear();
					this->cloneANDmove(m,&scan_2,w,a,rotate);
					this->matchScans((*m)[0],scan_2[0],&measurePairs);
					cout << "MeasurePairs Size: " << this->measurePairs.size() << endl;
					this->deletePixel(&demobild, &demodlg,0,m);
					this->visualRoboPose(&scan_2[0],&demobild,&demodlg,1);
					this->visualPairs(&measurePairs, &demobild, &demodlg);
					img::SaveBMP(demobild,"verschobeneScans.BMP");
					break;*/
				/*case 'g':
					cout << "case g" << endl;
					cout << "loesche gruene Pixel" << endl;
					this->deletePixel(&demobild, &demodlg,255,m);
					break;*/
				case 't':
					//cout << "ERstelle test Werte" << endl;
					//this->testMethods();
					//this->visualMeasures(&messungen, &bild, &bilddlg, true);
					//this->printMeasure(&messungen);
					break;
				case 'g':
					cout << "gruppiere Messung" << endl;
					this->groupMeasures();
					break;
				case 'b':
					cout << "Generiere Paare und matche" << endl;
					this->matching();
					break;
				case 'O':
					cout << "beende demoBewegung" << endl;
					demovar = false;
					break;
				case 'v':
					this->visualPairs(&measurePairs, &paarebild,&paardlg);
					break;
				default:
					key = '.';
					break;
			}
		}
	}
}
void mapping::deletePixel(img::ByteRGBImage *b, img::ByteRGBImageDialog *d, int value, deque<measurement> *m) {
	for(int i = 0; i<b->GetSizeX(); i++) {
		for(int j = 0; j<b->GetSizeY(); j++) {
			b->SetPixel(i,j,255);
		}
	}
	this->visualMeasures(m,b,d,true);
	d->ChangeImage((*b));
}
void mapping::deletePixel(img::ByteRGBImage *b, img::ByteRGBImageDialog *d) {
	for(int i = 0; i<b->GetSizeX(); i++) {
		for(int j = 0; j<b->GetSizeY(); j++) {
			b->SetPixel(i,j,255);
		}
	}
	d->ChangeImage((*b));
}
void mapping::map() {
	//char key;
	if(measurePairs.size() > 0) {
		cout << "Visualisiere Paare" << endl;
		this->visualPairs(&measurePairs,&demobild,&demodlg);
		img::SaveBMP(demobild,"two_diff_Scans_matched.BMP");
		return;
	}
	if((GroupScan_1.size() != 0) && (GroupScan_0.size() != 0)) {
		cout << "matching..." << endl;
		this->matchScans(GroupScan_0[0],GroupScan_1[0],&measurePairs);
		this->printPairs(&measurePairs);
		cout << "Matching beendet." << endl;
		return;
	}
	cout << "Scanne..." << endl;
	if(RohScan_0.size() < 5) {
		cout << "Speichere Scan_1, Anzahl: "<<RohScan_0.size() << endl;
		this->saveMeasure(p3dxptr, &RohScan_0);
		cout << "Scan beendet." << endl;
		return;
	} 
	if((RohScan_1.size() < 5) && (RohScan_0.size() == 5)) {
		cout << "SSpeichere Scan_1, Anzahl: "<<RohScan_1.size() << endl;
		this->saveMeasure(p3dxptr, &RohScan_1);
		cout << "Scan beendet." << endl;
		return;
	}
	if((RohScan_0.size() == 5) && (RohScan_1.size() == 5)) {
		cout << "Starte Optimierung" << endl;
		this->groupMeasures(&RohScan_0, &GroupScan_0);
		this->groupMeasures(&RohScan_1, &GroupScan_1);
		this->visualMeasures(&GroupScan_0,&bild, &bilddlg, 1);
		this->visualMeasures(&GroupScan_1,&bild, &bilddlg, 2);
		cout << "Optimierung beendet." << endl;
		return;
	}
	
		
		
}
