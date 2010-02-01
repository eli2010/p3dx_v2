#include "p3dxControls.h"


p3dxControls::p3dxControls(ArRobot *roboptr, mapping *mymap) {
	this->p3dxptr = roboptr;
	this->Rot_Value = 15;
	this->Vel_Value = 75;
	this->mapptr = mymap;
	this->Control();
}
void p3dxControls::lookAround() {
	//mapping maps(p3dxptr);
	while((this->p3dxptr->isConnected()) && (key == 'l')) {
		this->p3dxptr->comInt(ArCommands::VEL, Vel_Value);
		while((this->p3dxptr->getClosestSonarRange(-105, 105) > 500) && key == 'l') {
			//maps.startMapping((void *);
			//key = _getch_nolock();
			if(kbhit()) {
				key = _getch();
			}
		}
		this->p3dxptr->comInt(ArCommands::VEL, 0);
		//ArUtil::sleep(500);
		if(this->p3dxptr->getClosestSonarRange(1,105) <= 500) {
			this->p3dxptr->comInt(ArCommands::RVEL, -Rot_Value);
		}
		if(this->p3dxptr->getClosestSonarRange(-105,0) <= 500) {
			this->p3dxptr->comInt(ArCommands::RVEL, Rot_Value);
		}
		while((this->p3dxptr->getClosestSonarRange(-105, 105) <= 500) && key == 'l') {
			//maps.startMapping();
			//key = _getch_nolock();
			if(kbhit()) {
				key = _getch();
			}
		}
		//key = _getch_nolock();
		this->p3dxptr->comInt(ArCommands::RVEL, 0);
		ArUtil::sleep(500);
		//cout << "Weiter erkunden? druecke l" << endl;
	}
}
void p3dxControls::lookAroundFindBall() {
	//ballDetection detection(p3dxptr);
//	int ballseen = detection.moveToBall();
//	int rotatedToBall = detection.rotateToBall();
	while(this->p3dxptr->isConnected() && (key == 'l')) {
		this->p3dxptr->comInt(ArCommands::VEL, Vel_Value);
		while((this->p3dxptr->getClosestSonarRange(-105, 105) > 500)) { //|| (ballseen == 1)) {
			//ballseen = detection.moveToBall();
			//detection.reached();
		}
		this->p3dxptr->comInt(ArCommands::VEL, 0);
		//ArUtil::sleep(500);
		if(this->p3dxptr->getClosestSonarRange(1,105) <= 500) {
			this->p3dxptr->comInt(ArCommands::RVEL, -Rot_Value);
		}
		if(this->p3dxptr->getClosestSonarRange(-105,0) <= 500) {
			this->p3dxptr->comInt(ArCommands::RVEL, Rot_Value);
		}
		while(this->p3dxptr->getClosestSonarRange(-105, 105) <= 500) {
			//ballseen = detection.moveToBall();
			//detection.reached();
		}
		this->p3dxptr->comInt(ArCommands::RVEL, 0);
		ArUtil::sleep(500);
		//cout << "Weiter erkunden? druecke l" << endl;
		
		//}
	}
}
void p3dxControls::checkNarrow() {
	if((p3dxptr->getSonarRange(0) < 500 && p3dxptr->getSonarRange(1) < 500 && 
		p3dxptr->getSonarRange(7) < 500 && p3dxptr->getSonarRange(6) < 500 ) ) {
		cout << "NarrowPassage" <<endl;
	}
	else {
		cout << "keine NarrowPassage" <<endl;
	}
}
void p3dxControls::FindNarrowPassage() {
	//p3dxptr->comInt(ArCommands::VEL, 0);
	//ArUtil::sleep(500);
	//p3dxptr->comInt(ArCommands::RVEL, 0);
	//ArUtil::sleep(500);
	int sum = 0;
	ArPose currentPose = p3dxptr->getPose();
	for(int i = 0; i < 8; i++) {
		sum += p3dxptr->getSonarRange(i);
		cout << "SonarRange i: "<< i <<" "<<p3dxptr->getSonarRange(i)<<endl;
		cout << "Summe: "<<sum<<endl;
		
	}
	
	cout << "geteilte Summe" << sum/8 <<endl;
	//cout << currentPose.getTh() << endl;
}
void p3dxControls::Control() {
	char key_2;
	cout << "Tastatureingaben: " << endl;
	cout << "w: Vorwaerts; a: Linksdrehung; s: Rueckwaerts; d: Rechtdrehung" << endl;
	cout << "l: starten des LookaroundModus (Abbruch mit beliebiger Taste)" << endl;
	cout << "Aeenderungen der Geschwindigkeiten: " << endl;
	cout << "1 bzw 2 zum Erhoehen bzw Senken der Fahrgeschwindigkeit" << endl;
	cout << "3 bzw 4 zum Erhoehen bzw Senken der Drehgeschwindigkeit" << endl;
	cout << "o beendet das Programm" << endl;
	cout << "Start mit Vel = "<<Vel_Value<< " RVEL = " << Rot_Value << endl; 
	while(this->p3dxptr->isConnected()) {
		if(kbhit()) {
			key = _getch();
			switch(key) {
				case '3':
					this->Rot_Value += 5;
					cout << "Erhoehe Rot_Value: "<< Rot_Value << endl;
					break;
				case '4':
					this->Rot_Value -= 5;
					cout << "Senke Rot_Value: "<< Rot_Value << endl;
					break;
				case '1':
					this->Vel_Value += 5;
					cout << "Erhoehe Vel_Value: "<< Vel_Value << endl;
					break;
				case '2':
					this->Vel_Value -= 5;
					cout << "Senke Vel_Value: "<< Vel_Value << endl;
					break;
				case 'w':
					this->p3dxptr->comInt(ArCommands::RVEL, 0);
					if(Vel_Value < 0) {
						this->p3dxptr->comInt(ArCommands::VEL, -Vel_Value);	
					} else
						this->p3dxptr->comInt(ArCommands::VEL, Vel_Value);
					break;
				case 's':
					this->p3dxptr->comInt(ArCommands::RVEL, 0);
					this->p3dxptr->comInt(ArCommands::VEL, -Vel_Value);
					break;
				case 'a':
					this->p3dxptr->comInt(ArCommands::VEL, 0);
					this->p3dxptr->comInt(ArCommands::RVEL, Rot_Value);
					break;
				case 'd':
					this->p3dxptr->comInt(ArCommands::VEL, 0);
					this->p3dxptr->comInt(ArCommands::RVEL, -Rot_Value);
					break;
				case ' ':
					cout << "Emergency Stop" << endl;
					this->p3dxptr->comInt(ArCommands::VEL, 0);
					this->p3dxptr->comInt(ArCommands::RVEL, 0);
					break;
				case 'v':
					cout << "Rufe Visualisieren aus Mapping auf" << endl;
					this->mapptr->visualisieren();
					break;
				case 'g':
					cout << "gruppiere messungen" << endl;
					this->mapptr->groupMeasures();
					break;
				case 'l':
					cout << "Starte LookAround Modus" << endl;
					this->lookAround();
					break;
				case 'm':
					cout << "Berechne Mapping." << endl;
					key = ' ';
					this->mapptr->runMapping();
					break;
				case 'M':
					cout << "Map gespeichert" << endl;
					img::SaveBMP(this->mapptr->b, "map.BMP");
					break;
				case 'n':
					cout << "manuelles Mapping" << endl;
					this->mapptr->ManuellMapping();
					break;
				case 'o':
					this->p3dxptr->disconnect();
					break;
				case 'z':
					this->mapptr->map();
					break;
				case 'b':
					this->mapptr->matching();
					break;
				case 't':
					cout << "Informationen: " << endl;
					//cout << "Robo Position: " << endl;
					//cout << this->p3dxptr->getPose().getX() << " " << this->p3dxptr->getPose().getY() << " " << this->p3dxptr->getPose().getTh()<< endl;
					cout << "Batteriestatus getBatteryVoltage(): " << endl;
					cout << this->p3dxptr->getBatteryVoltage() << endl;
					cout << "Batteriestatus getRealBatteryVoltage(): " << endl;
					cout << this->p3dxptr->getRealBatteryVoltage() << endl;
					cout << "Batteriestatus getBatteryVoltageNow(): " << endl;
					cout << this->p3dxptr->getBatteryVoltageNow() << endl;

					//cout << "Gefahrene millimeter" << endl;
					//cout << this->p3dxptr->getOdometerDistance()<<endl;
					//cout << "Drehung in Grad" << endl;
					//cout << this->p3dxptr->getOdometerDegrees() << endl;

					/*cout << "SonarPositionen" << endl;
					for(int i = 0; i < anzsensor; i++) {
						cout << "Lokale Position von Sonar :"<<i<< " " <<p3dxptr->getSonarReading(i)->getSensorPosition().getX() << " " << p3dxptr->getSonarReading(i)->getSensorPosition().getY() << " " << p3dxptr->getSonarReading(i)->getSensorPosition().getTh() << endl;
						ArPose temp;
						temp.setPose(p3dxptr->getSonarReading(i)->getSensorPosition().getX(), p3dxptr->getSonarReading(i)->getSensorPosition().getY(), p3dxptr->getSonarReading(i)->getSensorPosition().getTh());
						ArTransform tmpTrans;
						tmpTrans.setTransform(p3dxptr->getPose());
						temp.setPose(tmpTrans.doTransform(temp));
						cout << "Globale Position von Sonar : "<<i<< " " << temp.getX() << " " << temp.getY() << " " << temp.getTh() << endl;
					}*/
					break;
				case 'p':
					this->mapptr->pseudoMove();
					break;
				default:
					key = '.';
					break;
			}
		}
	}
}
