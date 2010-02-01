#include "movement.h"
#include "ballDetection.h"
#include <iostream>
#include<conio.h>
#include <stdio.h>


double Rot_Value = 15;
double Vel_Value = 75;
char key;
//char yesno;

movement::movement(ArRobot *roboptr) {
	p3dxptr = roboptr;

}
void movement::lookAround() {
	while((p3dxptr->isConnected()) && (key == 'l')) {
		p3dxptr->comInt(ArCommands::VEL, Vel_Value);
		while((p3dxptr->getClosestSonarRange(-105, 105) > 500) && key == 'l') {
				//key = _getch_nolock();
			if(kbhit()) {
				key = _getch();
			}
		}
		p3dxptr->comInt(ArCommands::VEL, 0);
		//ArUtil::sleep(500);
		if(p3dxptr->getClosestSonarRange(1,105) <= 500) {
			p3dxptr->comInt(ArCommands::RVEL, -Rot_Value);
		}
		if(p3dxptr->getClosestSonarRange(-105,0) <= 500) {
			p3dxptr->comInt(ArCommands::RVEL, Rot_Value);
		}
		while((p3dxptr->getClosestSonarRange(-105, 105) <= 500) && key == 'l') {
			//key = _getch_nolock();
			if(kbhit()) {
				key = _getch();
			}
		}
		//key = _getch_nolock();
		p3dxptr->comInt(ArCommands::RVEL, 0);
		ArUtil::sleep(500);
		//cout << "Weiter erkunden? druecke l" << endl;
	}
}
void movement::lookAroundFindBall() {
	ballDetection detection(p3dxptr);
	int ballseen = detection.moveToBall();
	int rotatedToBall = detection.rotateToBall();
	while(p3dxptr->isConnected() && (key == 'l')) {
		p3dxptr->comInt(ArCommands::VEL, Vel_Value);
		while((p3dxptr->getClosestSonarRange(-105, 105) > 500) || (ballseen == 1)) {
			ballseen = detection.moveToBall();
			detection.reached();
		}
		p3dxptr->comInt(ArCommands::VEL, 0);
		//ArUtil::sleep(500);
		if(p3dxptr->getClosestSonarRange(1,105) <= 500) {
			p3dxptr->comInt(ArCommands::RVEL, -Rot_Value);
		}
		if(p3dxptr->getClosestSonarRange(-105,0) <= 500) {
			p3dxptr->comInt(ArCommands::RVEL, Rot_Value);
		}
		while(p3dxptr->getClosestSonarRange(-105, 105) <= 500) {
			ballseen = detection.moveToBall();
			detection.reached();
		}
		p3dxptr->comInt(ArCommands::RVEL, 0);
		ArUtil::sleep(500);
		//cout << "Weiter erkunden? druecke l" << endl;
		
		//}
	}
}
void movement::checkNarrow() {
	if((p3dxptr->getSonarRange(0) < 500 && p3dxptr->getSonarRange(1) < 500 && 
		p3dxptr->getSonarRange(7) < 500 && p3dxptr->getSonarRange(6) < 500 ) ) {
		cout << "NarrowPassage" <<endl;
	}
	else {
		cout << "keine NarrowPassage" <<endl;
	}
}
void movement::FindNarrowPassage() {
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
void movement::keyControl() {
	cout << "Tastatureingaben: " << endl;
	cout << "w: Vorwaerts; a: Linksdrehung; s: Rueckwaerts; d: Rechtdrehung" << endl;
	cout << "l: starten des LookaroundModus (Abbruch mit beliebiger Taste)" << endl;
	cout << "Aenderungen der Geschwindigkeiten: " << endl;
	cout << "1 bzw 2 zum Erhöhen bzw Senken der Fahrgeschwindigkeit" << endl;
	cout << "3 bzw 4 zum Erhöhen bzw Senken der Drehgeschwindigkeit" << endl;
	cout << "o beendet das Programm" << endl;
	cout << "Start mit Vel = "<<Vel_Value<< " RVEL = " << Rot_Value << endl; 
	while(p3dxptr->isConnected()) {
		//bool kbh = kbhit();
		if(kbhit()) {
			key = _getch();
			if(key == '3') {
				Rot_Value += 5;
				//cout << "setze Rot_Value+5" << endl;
				cout << "Erhoehe Rot_Value: "<< Rot_Value << endl;
				//if(yesno == 'y'
			}
			if(key == '4') {
				Rot_Value -= 5;
				//cout << "setze Rot_Value -5"<<endl;
				cout << "Senke Rot_Value: "<< Rot_Value << endl;
			}
			if(key == '1') {
				Vel_Value += 5;
				//cout << "Setze Vel_Value +5" << endl;
				cout << "Erhoehe Vel_Value: "<< Vel_Value << endl;
			}
			if(key == '2') {
				Vel_Value -= 5;
				//cout << "setze Vel_Value -5" << endl;
				cout << "Senke Vel_Value: "<< Vel_Value << endl;
			}
			if(key == 'w') {
				p3dxptr->comInt(ArCommands::RVEL, 0);
				//cout << "Fahre vorwaerts" << endl;
				if(Vel_Value < 0) {
					//cout << "Vel_Value negativ rechne *-1" << endl;
					p3dxptr->comInt(ArCommands::VEL, -Vel_Value);	
				} else
					p3dxptr->comInt(ArCommands::VEL, Vel_Value);
					//cout << "fahre normal" << endl;

			}
			if(key == 's') {
				p3dxptr->comInt(ArCommands::RVEL, 0);
				//cout << "rueckwaerts" << endl;
				//ArUtil::sleep(200);
				//cout << "Vel Value > 0 rechne *-1" << endl;
				p3dxptr->comInt(ArCommands::VEL, -Vel_Value);
			}
			if(key == 'a') {
				p3dxptr->comInt(ArCommands::VEL, 0);
				//cout << "drehe nach links" << endl;
				p3dxptr->comInt(ArCommands::RVEL, Rot_Value);
			}
			if(key == 'd') {
				p3dxptr->comInt(ArCommands::VEL, 0);
				//cout << "drehe nach rechts" << endl;
				p3dxptr->comInt(ArCommands::RVEL, -Rot_Value);
			}
			if(key == ' ') {
				cout << "Emergency Stop" << endl;
				p3dxptr->comInt(ArCommands::VEL, 0);
				p3dxptr->comInt(ArCommands::RVEL, 0);
			}
			if(key == 'l') {
				cout << "Starte LookAround Modus" << endl;
				movement::lookAround();
			}
			if(key == 'o') {
				p3dxptr->disconnect();
			}
			/*if(movement::checkWall()) {
				cout << "Automatischer Stop Grund: Wand" << endl;
				p3dxptr->comInt(ArCommands::VEL, 0);
				p3dxptr->comInt(ArCommands::RVEL, 0);
			}*/
		}
	}
	//cout << test << endl;
}
bool movement::checkWall() {
	//bool wand = false;
	for(int i = 0; i < 16; i++) {
		if(p3dxptr->getSonarRange(i) <= 500) {
			return true;
		}
	}
	return false;
}