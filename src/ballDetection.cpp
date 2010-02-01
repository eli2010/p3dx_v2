#include "ballDetection.h"

ballDetection::ballDetection(ArRobot *p3dx) {
	p3dxptr = p3dx;
	char c[50];
	for(int i = 0; i < 6; i++) {
		sprintf(c,"pics/Ballbild%d.bmp",i);
		img::LoadBMP(rgb, c);
		oldpics.push_back(rgb);
	}
	for(int i = 0; i < 6; i++) {
		sprintf(c, "pics/newBallbild%d.bmp",i);
		img::LoadBMP(rgb,c);
		refpics.push_back(rgb);
	}
	detec.GenerateLUT(oldpics, refpics);
	initialGrabber();
}

double ballDetection::findObj() {
	int countTmp = 0;
	int countMax = 0;
	KoordX = -1;
	KoordY = -1;
	int KoordXTmp = -1;
	int KoordYTmp = -1;
	//countTmp = -1;
	//countMax = -1;
	Sleep(500);
	grabber.NextImg();
	memcpy(rgb.GetImagePtr(), grabber.GetImgData(), width*heigth*3);
	rgb.SwapImage(detec.Detect(rgb));
	rgbdlg.ChangeImage(rgb);
	for(int i = 0; i <= rgb.GetSizeX()-windowSize-1; i = i + windowSize) {
		for(int j = 0; j <= rgb.GetSizeY()-windowSize-1; j = j + windowSize) {
			countTmp = 0;
			for(int x = i; x < i + windowSize; x++) {
				for(int y = j; y < j + windowSize; y++) {
					int blue = rgb.GetPixel(x,y).blue;
					int red = rgb.GetPixel(x,y).red;
					int green = rgb.GetPixel(x,y).green;
					if((blue != 0) && (red != 0) && (green != 0)) {
						countTmp++;

					}
				}
			}

			//cout<<"TMP Count: "<<countTmp<<" CountMax: "<<countMax<<endl;
			if(countTmp > countMax) {
				//cout<<"Tausche Max und Tmp"<<endl;
				countMax = countTmp;
				KoordXTmp = i;
				KoordYTmp = j;
				KoordX = i+(windowSize/2);
				KoordY = j+(windowSize/2);
			}
		}
	}
	double XMax = -1;
	double YMax = -1;
	double XMin = 5000;
	double YMin = 5000;
	/*int XMaxTmp = -1;
	int XMinTmp = 5000;
	int YMaxTmp = -1;
	int YMinTmp = 5000;*/
	if(KoordX >= 0 && KoordXTmp >= 0 && KoordY >= 0 && KoordYTmp >= 0) {
		for(int i = KoordXTmp; i < KoordXTmp + windowSize; i++) {
			for(int j = KoordYTmp; j < KoordYTmp + windowSize; j++) {
				int blue = rgb.GetPixel(i,j).blue;
				int red = rgb.GetPixel(i,j).red;
				int green = rgb.GetPixel(i,j).green;
				if((blue != 0) && (red != 0) && (green != 0)) {
					if(i > XMax) {
						XMax = i;
					}
					if(i < XMin) {
						XMin = i;
					}
					if(j > YMax) {
						YMax = j;
					}
					if(j < YMin) {
						YMin = j;
					}
				}
			}
		}
	}
	//cout << "XMax: "<<XMax<<"XMin: "<<XMin<<"YMax: "<<YMax<<"YMin: "<<YMin<<endl;
	double PixelCount = 0;
	if((XMax >= 0) && (XMin >= 0) && (YMax >= 0) && (YMin >= 0)) {
		for(int i = XMin; i <= XMax; i++) {
			for(int j = YMin; j <= YMax; j++) {
				int blue = rgb.GetPixel(i,j).blue;
				int red = rgb.GetPixel(i,j).red;
				int green = rgb.GetPixel(i,j).green;
				if((blue != 0) && (red != 0) && (green != 0)) {
					PixelCount++;
				}
			}
		}
	}

	double rate = PixelCount/((XMax-XMin)*(YMax-YMin));
	if(rate < 0 || rate > 1) {
		cout << "Rate auf 0 gesetzt" << endl;
		rate = 0;
	}
	cout << "Pixel/((XMax-XMin)*(YMax-YMin)): " <<rate<<endl;
	//cout << "maximale Anzahl von Gruenen Punkten: " << countMax <<" bei x: "<<KoordX<<" und y: "<<KoordY<<endl;
	Sleep(10);
	return rate;
}

int ballDetection::moveToBall() {
	cout << "rufe Movetoball auf"<<endl;
	//cout << "KoordX: "<< KoordX<<endl;
	//int headingCount = 0;
	double obj = findObj();
	// kleiner wählen damit auf ball in größerer Entfernung gefunden wird
	while(obj >= 0.5) {
		p3dxptr->comInt(ArCommands::VEL, 0);
		//ballimBild = true;
		//cout << "#Drehungen: "<< headingCount << endl;

		ArPose currentPose = p3dxptr->getPose();
		cout<<"Koorigiere Heading "<<currentPose.getTh()<<endl;
		if(KoordX >= 0 && KoordX <= 399) {
			//p3dx.comInt(ArCommands::HEAD, i);
			cout<<"Setze Heading -10 "<<endl;
			//			headingCount++;
			p3dxptr->comInt(ArCommands::HEAD, (currentPose.getTh()+5));
			ArUtil::sleep(500);

		}

		if(KoordX > 399 && KoordX <= 799) {

			//p3dx.comInt(ArCommands::HEAD, i);
			cout<<"Setze Heading +10 "<<endl;
			//			headingCount++;
			p3dxptr->comInt(ArCommands::HEAD, currentPose.getTh()-5);
			ArUtil::sleep(500);


		}
		if(rotateToBall() == 1) {
			cout << "ball in der Mitte" <<endl;
			p3dxptr->comInt(ArCommands::VEL, 75);
			reached();
			cout << "Rückgabe Wert 1" << endl;
			return 1;
		}else {
			p3dxptr->comInt(ArCommands::VEL, 75);
			p3dxptr->comInt(ArCommands::RVEL, 0);
			cout << "Rückgabe Wert 0" << endl;
			return 0;
		}
	}
	if(obj < 0.5) {
		return 0;
	}	
}


int ballDetection::rotateToBall() {
	//cout << "rufe rotate to Ball auf" <<endl;
	int count = 0;
	int TmpCount = 0;
	//Sleep(500);
	grabber.NextImg();
	memcpy(rgb.GetImagePtr(), grabber.GetImgData(), width*heigth*3);
	rgb.SwapImage(detec.Detect(rgb));
	rgbdlg.ChangeImage(rgb);
	for(int j = 0; j <= rgb.GetSizeY()-windowSize-1; j= j+windowSize) {
		TmpCount = 0;
		for(int i = 349; i < 449; i++) {
			for(int x = j; x < j + windowSize; x++) {
				int blue = rgb.GetPixel(i,x).blue;
				int red = rgb.GetPixel(i,x).red;
				int green = rgb.GetPixel(i,x).green;
				if((blue != 0) && (red != 0) && (green != 0)) {
					TmpCount++;
				}

				if(TmpCount > count) {
					count = TmpCount;
				}
			}
		}

	}
	cout <<" Rotate Count: "<< count <<endl;
	if(count >= 1200) {
		return 1;
	} else 
		return 0;
}
void ballDetection::reached() {
	if(checkball() == 1) {
		cout << "Ziel erreicht disconnect" <<endl;
		p3dxptr->disconnect();
		grabber.Stop();
		detec.~SkinDetection();
		grabber.~Grabber();
		exit(0);
	}	
}
void ballDetection::initialGrabber() {
	grabconfig.source.id = -1;
	grabconfig.source.type = vid::EnumSourceType::SOURCE_TYPE_LIVE;
	grabconfig.source.name = "Logitech QuickCam Pro 9000";
	vid::Format format;
	format.fps = 20;
	format.height = heigth;
	format.width = width;
	grabconfig.formats.push_back(format);
	HRESULT hr = grabber.OpenConfiguration(grabconfig);
	rgb.SetSize(width, heigth);
	grabber.Run();
}



int ballDetection::checkball() {
	grabber.NextImg();
	memcpy(rgb.GetImagePtr(), grabber.GetImgData(), width*heigth*3);
	rgb.SwapImage(detec.Detect(rgb));
	rgbdlg.ChangeImage(rgb);
	int TmpCount = 0;
	for(int i = 0; i < rgb.GetSizeX(); i++) {
		for(int j = 0; j < rgb.GetSizeY(); j++) {
			int blue = rgb.GetPixel(i,j).blue;
			int red = rgb.GetPixel(i,j).red;
			int green = rgb.GetPixel(i,j).green;
			if((blue != 0) && (red != 0) && (green != 0)) {
				TmpCount++;
			}
		}
	}
	//cout << "Check Ball Count" << TmpCount << endl;
	if(TmpCount > 85000) {
		return 1;
	} else
		return 0;
}